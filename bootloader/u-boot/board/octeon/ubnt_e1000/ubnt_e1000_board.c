/*
 * (C) Copyright 2015 Cavium Inc. <support@cavium.com>
 *
 * Copyright (C) 2016 Ubiquiti Networks, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include <asm/mipsregs.h>
#include <asm/arch/octeon_boot.h>
#include <asm/arch/cvmx-sim-magic.h>
#include <asm/arch/cvmx-qlm.h>
#include <asm/arch/octeon_fdt.h>
#include <fdt_support.h>
#include <asm/arch/octeon_board_common.h>
#include <asm/arch/lib_octeon.h>
#include <asm/arch/lib_octeon_shared.h>
#include <asm/arch/octeon_qlm.h>
#include <asm/arch/cvmx-helper-util.h>
#include <asm/arch/cvmx-helper-cfg.h>
#include <asm/arch/cvmx-smi-defs.h>
#include <asm/arch/cvmx-mdio.h>
#include <asm/gpio.h>

#include "ubnt_e1000_board.h"

struct cregs adt7475[] = {
	// set to automatic mode
	{ENHANCE_ACOUS1_REG,          0xe},
	// configuring for 4-wire fan (freq: 22.5khz)
	{REMOTE1_TR_PWM1_FREQ_REG,   0xcc},
	{LOCAL_TR_PWM2_FREQ_REG,     0xcc},
	{REMOTE2_TR_PWM3_FREQ_REG,   0xcc},
	// RMT_TEMP1: Tmin=60C
	{REMOTE1_TEMP_TMIN_REG,      0x7c},
	{REMOTE1_THERM_TEMP_LMT_REG, 0xa4},
	// PWM1/PWM2/PWM3 Min Duty Cycle=1/4
	{PWM1_MIN_DUTY_REG,          0x40},
	{PWM2_MIN_DUTY_REG,          0x40},
	{PWM3_MIN_DUTY_REG,          0x40},
	// Automatic mode: RMT_TEMP1 tie to PWM1/PWM2/PWM3
	{PWM1_CONFIG_REG,             0x2},
	{PWM2_CONFIG_REG,             0x2},
	{PWM3_CONFIG_REG,             0x2}
};

extern void print_isp_volt(const char **labels, uint16_t isp_dev_addr, uint8_t adc_chan);

extern int read_ispPAC_mvolts(uint16_t isp_dev_addr, uint8_t adc_chan);

extern int read_ispPAC_mvolts_avg(int loops, uint16_t isp_dev_addr, uint8_t adc_chan);

DECLARE_GLOBAL_DATA_PTR;

/*
 * parse_env_var:	Parse the environment variable ("bgx_for_mix%d") to
 *			extract the lmac it is set to.
 *
 *  index:		Index of environment variable to parse.
 *			environment variable.
 *  env_bgx:		Updated with the bgx of the lmac in the environment
 *			variable.
 *  env_lmac:		Updated with the index of lmac in the environment
 *			variable.
 *
 *  returns:		Zero on success, error otherwise.
 */
static int parse_env_var(int index, int *env_bgx, int *env_lmac)
{
	char	env_var[20];
	ulong	xipd_port;

	sprintf(env_var, "bgx_for_mix%d", index);
	if ((xipd_port = getenv_ulong(env_var, 0, 0xffff)) != 0xffff) {
		int xiface;
		struct cvmx_xiface xi;
		struct cvmx_xport xp;

		/*
		 * The environemt variable is set to the xipd port. Convert the
		 * xipd port to numa node, bgx, and lmac.
		 */
		xiface = cvmx_helper_get_interface_num(xipd_port);
		xi = cvmx_helper_xiface_to_node_interface(xiface);
		xp = cvmx_helper_ipd_port_to_xport(xipd_port);
		*env_bgx = xi.interface;
		*env_lmac = cvmx_helper_get_interface_index_num(xp.port);
		return 0;
	}

	return -1;
}

/*
 * get_lmac_fdt_node:	Search the device tree for the node corresponding to
 *			a given bgx lmac.
 *
 *  fdt:		Pointer to flat device tree
 *  search_node:	Numa node of the lmac to search for.
 *  search_bgx:		Bgx of the lmac to search for.
 *  search_lmac:	Lmac index to search for.
 *  compat:		Compatible string to search for.

 *  returns:		The device tree node of the lmac if found,
 *			or -1 otherwise.
 */
static int get_lmac_fdt_node(const void *fdt, int search_node, int search_bgx,
			     int search_lmac, const char *compat)
{
	int		node;
	const fdt32_t	*reg;
	u64		addr;
	int		fdt_node = -1;
	int		fdt_bgx = -1;
	int		fdt_lmac = -1;
	int		len;
	int		parent;

	/* Iterate through all bgx ports */
	node = -1;
	while ((node = fdt_node_offset_by_compatible((void *)fdt, node,
						     compat)) >= 0) {
		/* Get the node and bgx from the physical address */
		if (((parent = fdt_parent_offset(fdt, node)) < 0) ||
		    !(reg = fdt_getprop(fdt, parent, "reg", &len)))
			continue;

		addr = fdt_translate_address((void *)fdt, parent, reg);
		fdt_node = (addr >> 36) & 0x7;
		fdt_bgx = (addr >> 24) & 0xf;

		/* Get the lmac index from the reg property */
		if ((reg = fdt_getprop(fdt, node, "reg", &len)))
			fdt_lmac = *reg;

		/* Check for a match */
		if (search_node == fdt_node && search_bgx == fdt_bgx &&
		    search_lmac == fdt_lmac)
			return node;
	}

	return -1;
}

/*
 * get_mix_fdt_node:	Search the device tree for the node corresponding to
 *			a given mix.
 *
 *  fdt:		Pointer to flat device tree
 *  search_node:	Mix numa node to search for.
 *  search_index:	Mix index to search for.
 *
 *  returns:		The device tree node of the lmac if found,
 *			or -1 otherwise.
 */
static int get_mix_fdt_node(const void *fdt, int search_node, int search_index)
{
	int		node;

	/* Iterate through all the mix fdt nodes */
	node = -1;
	while ((node = fdt_node_offset_by_compatible((void *)fdt, node,
					"cavium,octeon-7890-mix")) >= 0) {
		int		parent;
		int		len;
		const char	*name;
		int		mix_numa_node;
		const fdt32_t	*reg;
		int		mix_index = -1;
		u64		addr;

		/* Get the numa node of the mix from the parent node name */
		if (((parent = fdt_parent_offset(fdt, node)) < 0) ||
		    ((name = fdt_get_name(fdt, parent, &len)) == NULL) ||
		    ((name = strchr(name, '@')) == NULL))
			continue;

		name++;
		mix_numa_node = simple_strtol(name, NULL, 0) ? 1 : 0;

		/* Get the mix index from the reg property */
		if ((reg = fdt_getprop(fdt, node, "reg", &len))) {
			addr = fdt_translate_address((void *)fdt, parent, reg);
			mix_index = (addr >> 11) & 1;
		}

		/* Check for a match */
		if (mix_numa_node == search_node && mix_index == search_index)
			return node;
	}

	return -1;
}

/*
 * fdt_fix_mix:		Fix the mix nodes in the device tree. Only the mix nodes
 *			configured by the user will be preserved. All other mix
 *			nodes will be trimmed.
 *
 *  fdt:		Pointer to flat device tree
 *
 *  returns:		Zero on success, error otherwise.
 */
static int fdt_fix_mix(const void *fdt)
{
	int	node;
	int	next_node;
	int	len;
	int	i;

	/* Parse all the mix port environment variables */
	for (i = 0; i < MAX_MIX_ENV_VARS; i++) {
		int	env_node = 0;
		int	env_bgx = -1;
		int	env_lmac = -1;
		int	lmac_fdt_node = -1;
		int	mix_fdt_node = -1;
		int	lmac_phandle;
		char	*compat;

		/* Get the lmac for this environment variable */
		if (parse_env_var(i, &env_bgx, &env_lmac))
			continue;

		/* Get the fdt node for this lmac and add a phandle to it */
		compat = "cavium,octeon-7890-bgx-port";
		if ((lmac_fdt_node = get_lmac_fdt_node(fdt, env_node, env_bgx,
						       env_lmac, compat)) < 0) {
			/* Must check for the xcv compatible string too */
			compat = "cavium,octeon-7360-xcv";
			if ((lmac_fdt_node = get_lmac_fdt_node(fdt, env_node,
							       env_bgx,
							       env_lmac,
							       compat)) < 0) {
				printf("WARNING: Failed to get lmac fdt node "
				       "for %d%d%d\n", env_node, env_bgx,
				       env_lmac);
			continue;
			}
		}

		lmac_phandle = fdt_alloc_phandle((void *)fdt);
		fdt_set_phandle((void *)fdt, lmac_fdt_node, lmac_phandle);

		/* Get the fdt mix node corresponding to this lmac */
		if ((mix_fdt_node = get_mix_fdt_node(fdt, env_node,
						     env_lmac)) < 0)
			continue;

		/* Point the mix to the lmac */
		fdt_getprop(fdt, mix_fdt_node, "cavium,mac-handle", &len);
		fdt_setprop_inplace((void *)fdt, mix_fdt_node,
				    "cavium,mac-handle", &lmac_phandle, len);
	}

	/* Trim unused mix'es from the device tree */
	for (node = fdt_next_node(fdt, -1, NULL); node >= 0;
	     node = next_node) {
		const char	*compat;
		const fdt32_t	*reg;

		next_node = fdt_next_node(fdt, node, NULL);

		if ((compat = fdt_getprop(fdt, node, "compatible", &len))) {
			if (strcmp(compat, "cavium,octeon-7890-mix"))
				continue;

			if ((reg = fdt_getprop(fdt, node,
					       "cavium,mac-handle", &len))) {
				if (*reg == 0xffff)
					fdt_nop_node((void *)fdt, node);
			}
		}
	}

	return 0;
}

int no_phy[8] = {0, 0, 0, 0, 0, 0, 0, 0};

static void kill_fdt_phy(void *fdt, int offset, void *arg)
{
	int len, phy_offset;
	const fdt32_t *php;
	uint32_t phandle;

	php = fdt_getprop(fdt, offset, "phy-handle", &len);
	if (php && len == sizeof(*php)) {
		phandle = fdt32_to_cpu(*php);
		fdt_nop_property(fdt, offset, "phy-handle");
		phy_offset = fdt_node_offset_by_phandle(fdt, phandle);
		if (phy_offset > 0)
			fdt_nop_node(fdt, phy_offset);
	}
}

void __fixup_xcv(void)
{
	unsigned long bgx = getenv_ulong("bgx_for_rgmii", 10, (unsigned long)-1);
	char fdt_key[16];
	int i;

	debug("%s: BGX %d\n", __func__, (int)bgx);

	for (i = 0; i < 3; i++) {
		snprintf(fdt_key, sizeof(fdt_key),
			 bgx == i ? "%d,xcv" : "%d,not-xcv", i);
		debug("%s: trimming bgx %lu with key %s\n",
		      __func__, bgx, fdt_key);

		octeon_fdt_patch_rename((void *)gd->fdt_blob, fdt_key,
					"cavium,xcv-trim", true, NULL, NULL);
	}
}

/* QLM0 - QLM6 */
void __fixup_fdt(void)
{
	int qlm;
	int speed = 0;

	for (qlm = 0; qlm < 7; qlm++) {
		enum cvmx_qlm_mode mode;
		char fdt_key[16];
		const char *type_str = "none";
		mode = cvmx_qlm_get_mode(qlm);
		switch (mode) {
		case CVMX_QLM_MODE_SGMII:
		case CVMX_QLM_MODE_RGMII_SGMII:
		case CVMX_QLM_MODE_RGMII_SGMII_1X1:
			type_str = "sgmii";
			break;
		case CVMX_QLM_MODE_XAUI:
		case CVMX_QLM_MODE_RGMII_XAUI:
			speed = (cvmx_qlm_get_gbaud_mhz(qlm) * 8 / 10) * 4;
			if (speed == 10000)
				type_str = "xaui";
			else
				type_str = "dxaui";
			break;
		case CVMX_QLM_MODE_RXAUI:
		case CVMX_QLM_MODE_RGMII_RXAUI:
			type_str = "rxaui";
			break;
		case CVMX_QLM_MODE_XLAUI:
		case CVMX_QLM_MODE_RGMII_XLAUI:
			type_str = "xlaui";
			break;
		case CVMX_QLM_MODE_XFI:
		case CVMX_QLM_MODE_RGMII_XFI:
		case CVMX_QLM_MODE_RGMII_XFI_1X1:
			type_str = "xfi";
			break;
		case CVMX_QLM_MODE_10G_KR:
		case CVMX_QLM_MODE_RGMII_10G_KR:
			type_str = "10G_KR";
			break;
		case CVMX_QLM_MODE_40G_KR4:
		case CVMX_QLM_MODE_RGMII_40G_KR4:
			type_str = "40G_KR4";
			break;
		case CVMX_QLM_MODE_SATA_2X1:
			type_str = "sata";
			break;
		case CVMX_QLM_MODE_SGMII_2X1:
		case CVMX_QLM_MODE_XFI_1X2:
		case CVMX_QLM_MODE_10G_KR_1X2:
		case CVMX_QLM_MODE_RXAUI_1X2:
		case CVMX_QLM_MODE_MIXED: // special for DLM5 & DLM6
		{
			cvmx_bgxx_cmrx_config_t cmr_config;
			cvmx_bgxx_spux_br_pmd_control_t pmd_control;
			int mux = cvmx_qlm_mux_interface(2);
			if (mux == 2) {   // only dlm6
				cmr_config.u64 = cvmx_read_csr(
						CVMX_BGXX_CMRX_CONFIG(2, 2));
				pmd_control.u64 = cvmx_read_csr(
						CVMX_BGXX_SPUX_BR_PMD_CONTROL(2, 2));
			} else {
				if (qlm == 5) {
					cmr_config.u64 = cvmx_read_csr(
							CVMX_BGXX_CMRX_CONFIG(0, 2));
					pmd_control.u64 = cvmx_read_csr(
							CVMX_BGXX_SPUX_BR_PMD_CONTROL(0, 2));
				} else {
					cmr_config.u64 = cvmx_read_csr(
							CVMX_BGXX_CMRX_CONFIG(2, 2));
					pmd_control.u64 = cvmx_read_csr(
							CVMX_BGXX_SPUX_BR_PMD_CONTROL(2, 2));
				}
			}
			switch (cmr_config.s.lmac_type) {
			case 0:
				type_str = "sgmii";
				break;
			case 1:
				type_str = "xaui";
				break;
			case 2:
				type_str = "rxaui";
				break;
			case 3:
				if (pmd_control.s.train_en)
					type_str = "10G_KR";
				else
					type_str = "xfi";
				break;
			case 4:
				if (pmd_control.s.train_en)
					type_str = "40G_KR4";
				else
					type_str = "xlaui";
				break;
			default:
				type_str = "none";
				break;
			}
			break;
		}
		default:
			type_str = "none";
			break;
		}
		sprintf(fdt_key, "%d,%s", qlm, type_str);
		debug("Patching qlm %d for %s for mode %d%s\n",
		      qlm, fdt_key, mode, no_phy[qlm] ? ", removing PHY" : "");
		octeon_fdt_patch_rename((void *)gd->fdt_blob, fdt_key, NULL,
					true, no_phy[qlm] ? kill_fdt_phy : NULL, NULL);
	}
}

int board_fixup_fdt(void)
{
	__fixup_fdt();
	__fixup_xcv();

	/* Fix the mix ports */
	fdt_fix_mix(gd->fdt_blob);

	return 0;
}

/* Raise an integer to a power */
static uint64_t ipow(uint64_t base, uint64_t exp)
{
	uint64_t result = 1;
	while (exp) {
		if (exp & 1)
			result *= base;
		exp >>= 1;
		base *= base;
	}
	return result;
}

/*
 * This function now supports only single (node0) configuration.
 * Some of the called funcs (read_ispPAC_mvolts_avg(), octeon_led_str_write())
 * are hard-coded for node0. Also we set gd->arch.mcu_rev_maj, ...
 * It can be extended to support multinode if needed/important
 */
static int checkboardinfo(void)
{
        int core_mVolts, dram_mVolts0, dram_mVolts1;
	char mcu_ip_msg[64] = { 0 };
	char tmp[10];
	int characters, idx = 0, value = 0;

	debug("In %s\n", __func__);
	if (octeon_show_info()) {

		int mcu_rev_maj = 0;
		int mcu_rev_min = 0;

		if (twsii_mcu_read(0x00) == 0xa5
		    && twsii_mcu_read(0x01) == 0x5a) {
			gd->arch.mcu_rev_maj = mcu_rev_maj = twsii_mcu_read(2);
			gd->arch.mcu_rev_min = mcu_rev_min = twsii_mcu_read(3);
		} else {
                    return -1;     /* Abort if we can't access the MCU */
                }

		core_mVolts  = read_ispPAC_mvolts_avg(10, BOARD_ISP_TWSI_ADDR, 8);
		dram_mVolts0 = read_ispPAC_mvolts_avg(10, BOARD_ISP_TWSI_ADDR, 7); /* DDR0 */
		dram_mVolts1 = read_ispPAC_mvolts_avg(10, BOARD_ISP_TWSI_ADDR, 6); /* DDR1 */

		if (twsii_mcu_read(0x14) == 1)
			sprintf(mcu_ip_msg, "MCU IPaddr: %d.%d.%d.%d, ",
				twsii_mcu_read(0x10),
				twsii_mcu_read(0x11),
				twsii_mcu_read(0x12),
				twsii_mcu_read(0x13));
		printf("MCU rev: %d.%02d, %sCPU voltage: %d.%03d DDR{0,1} voltages: %d.%03d,%d.%03d\n",
		       gd->arch.mcu_rev_maj, gd->arch.mcu_rev_min, mcu_ip_msg,
		       core_mVolts  / 1000, core_mVolts  % 1000,
		       dram_mVolts0 / 1000, dram_mVolts0 % 1000,
		       dram_mVolts1 / 1000, dram_mVolts1 % 1000);

#define LED_CHARACTERS 8
		value = core_mVolts;
		idx = sprintf(tmp, "%lu ", gd->cpu_clk/(1000*1000));
		characters = LED_CHARACTERS - idx;

		if (value / 1000) {
			idx += sprintf(tmp + idx, "%d", value / 1000);
			characters = LED_CHARACTERS - idx;
		}

		characters -= 1;	/* Account for decimal point */

		value %= 1000;
		value = DIV_ROUND_UP(value, ipow(10, max(3 - characters, 0)));

		idx += sprintf(tmp + idx, ".%0*d", min(3, characters), value);

		/* Display CPU speed and voltage on display */
		octeon_led_str_write(tmp);
	} else {
		octeon_led_str_write("Boot    ");
	}

	return 0;
}

/**
 * Here is the description of the parameters that are passed to QLM configuration
 * 	param0 : The QLM to configure
 * 	param1 : Speed to configure the QLM at
 * 	param2 : Mode the QLM to configure
 * 	param3 : 1 = RC, 0 = EP
 * 	param4 : 0 = GEN1, 1 = GEN2, 2 = GEN3
 * 	param5 : ref clock select, 0 = 100Mhz, 1 = 125MHz, 2 = 156MHz
 * 	param6 : ref clock input to use:
 * 		 0 - external reference (QLMx_REF_CLK)
 * 		 1 = common clock 0 (QLMC_REF_CLK0)
 * 		 2 = common_clock 1 (QLMC_REF_CLK1)
 */
void board_configure_qlms(void)
{
	int qlm;
	char env_var[16];
	int speed[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	int mode[8] = { -1, -1, -1, -1, -1, -1, -1, -1 };
	int pcie_rc[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	int pcie_gen[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	int ref_clock_sel[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	int ref_clock_input[8] = {0, 0, 2, 2, 0, 0, 0, 0}; //156.25MHZ comes from QLMC_REF_CLK1
	int rbgx, rqlm;

	octeon_init_qlm(0);

	gpio_direction_output(27, 0);	/* Put RGMII PHY in reset */

#if defined(CONFIG_OCTEON_MCU_PROBE) && defined(BOARD_MCU_TWSI_ADDR)
	if (octeon_mcu_probe(0) != -1) {
		return 0;
	}
#endif
	rbgx = getenv_ulong("bgx_for_rgmii", 10, (unsigned long)-1);
	switch(rbgx) {
	case 0:
		rqlm = 2;
		break;
	case 1:
		rqlm = 3;
		break;
	case 2:
		rqlm = 5;
		break;
	default:
		rqlm = -1;
		break;
	}
	for (qlm = 0; qlm < 7; qlm++) {
		const char *mode_str;
		char spd_env[16];

		mode[qlm] = CVMX_QLM_MODE_DISABLED;
		sprintf(env_var, "qlm%d_mode", qlm);
		mode_str = getenv(env_var);
		if (!mode_str)
			continue;

		if (qlm == 4 &&
		    mode[4] != -1 &&
		    mode[4] != CVMX_QLM_MODE_SATA_2X1) {
			printf("Error: DLM 4 can only be configured for SATA\n");
			continue;
		}

		if (strstr(mode_str, ",no_phy"))
			no_phy[qlm] = 1;

		if (!strncmp(mode_str, "sgmii", 5)) {
			bool rgmii = false;
			speed[qlm] = 1250;
			if (rqlm == qlm && qlm < 5) {
				mode[qlm] = CVMX_QLM_MODE_RGMII_SGMII;
				rgmii = true;
			} else if (qlm == 6 || qlm == 5) {
				if (rqlm == qlm && qlm == 5) {
					mode[qlm] = CVMX_QLM_MODE_RGMII_SGMII_1X1;
					rgmii = true;
				} else if (rqlm == 5 && qlm == 6
					   && mode[5] != CVMX_QLM_MODE_RGMII_SGMII_1X1) {
					mode[qlm] = CVMX_QLM_MODE_RGMII_SGMII_2X1;
					rgmii = true;
				} else {
					mode[qlm] = CVMX_QLM_MODE_SGMII_2X1;
				}
			} else {
				mode[qlm] = CVMX_QLM_MODE_SGMII;
			}
			ref_clock_sel[qlm] = 2;
			if (qlm == 5 || qlm == 6)
				ref_clock_input[qlm] = 2; // use QLMC_REF_CLK1
			if (no_phy[qlm]) {
				int i;
				int start = 0, stop = 2;
				rbgx = 0;
				switch (qlm) {
				case 3:
					rbgx = 1;
				case 2:
					for (i = 0; i < 4; i++) {
						printf("Ignoring PHY for interface: %d, port: %d\n",
						       rbgx, i);
						cvmx_helper_set_port_force_link_up(rbgx, i, true);
					}
					break;
				case 6:
					start = 2;
					stop = 4;
				case 5:

					for (i = start; i < stop; i++) {
						printf("Ignoring PHY for interface: %d, port: %d\n",
						       2, i);
						cvmx_helper_set_port_force_link_up(2, i, true);
					}
					break;
				default:
					printf("SGMII not supported for QLM/DLM %d\n",
					       qlm);
					break;
				}
			}
			printf("QLM %d: SGMII%s\n", qlm, rgmii ? ", RGMII" : "");
		} else if (!strncmp(mode_str, "xaui", 4)) {
			speed[qlm] = 3125;
			mode[qlm] = CVMX_QLM_MODE_XAUI;
			ref_clock_sel[qlm] = 2;
			if (qlm == 5 || qlm == 6)
				ref_clock_input[qlm] = 2; // use QLMC_REF_CLK1
			printf("QLM %d: XAUI\n", qlm);
		} else if (!strncmp(mode_str, "dxaui", 5)) {
			speed[qlm] = 6250;
			mode[qlm] = CVMX_QLM_MODE_XAUI;
			ref_clock_sel[qlm] = 2;
			if (qlm == 5 || qlm == 6)
				ref_clock_input[qlm] = 2; // use QLMC_REF_CLK1
			printf("QLM %d: DXAUI\n", qlm);
		} else if (!strncmp(mode_str, "rxaui", 5)) {
			bool rgmii = false;
			speed[qlm] = 6250;
			if (qlm == 5 || qlm == 6) {
				if (rqlm == qlm && qlm == 5) {
					mode[qlm] = CVMX_QLM_MODE_RGMII_RXAUI;
					rgmii = true;
				} else {
					mode[qlm] = CVMX_QLM_MODE_RXAUI_1X2;
				}
			} else {
				mode[qlm] = CVMX_QLM_MODE_RXAUI;
			}
			ref_clock_sel[qlm] = 2;
			if (qlm == 5 || qlm == 6)
				ref_clock_input[qlm] = 2; // use QLMC_REF_CLK1
			printf("QLM %d: RXAUI%s\n", qlm, rgmii ? ", rgmii" : "");
		} else if (!strncmp(mode_str, "xlaui", 5)) {
			speed[qlm] = 103125;
			mode[qlm] = CVMX_QLM_MODE_XLAUI;
			ref_clock_sel[qlm] = 2;
			if (qlm == 5 || qlm == 6)
				ref_clock_input[qlm] = 2; // use QLMC_REF_CLK1
			sprintf(spd_env, "qlm%d_speed", qlm);
			if (getenv(spd_env)) {
				int spd = getenv_ulong(spd_env, 0, 8);
				if (spd)
					speed[qlm] = spd;
				else
					speed[qlm] = 103125;
			}
			printf("QLM %d: XLAUI\n", qlm);
		} else if (!strncmp(mode_str, "xfi", 3)) {
			bool rgmii = false;
			speed[qlm] = 103125;
			if (rqlm == qlm) {
				mode[qlm] = CVMX_QLM_MODE_RGMII_XFI;
				rgmii = true;
			} else if (qlm == 5 || qlm == 6) {
				mode[qlm] = CVMX_QLM_MODE_XFI_1X2;
			} else {
				mode[qlm] = CVMX_QLM_MODE_XFI;
			}
			ref_clock_sel[qlm] = 2;
			if (qlm == 5 || qlm == 6)
				ref_clock_input[qlm] = 2; // use QLMC_REF_CLK1
			printf("QLM %d: XFI%s\n", qlm, rgmii ? ", RGMII" : "");
		} else if (!strncmp(mode_str, "10G_KR", 6)) {
			speed[qlm] = 103125;
			if (rqlm == qlm && qlm == 5)
				mode[qlm] = CVMX_QLM_MODE_RGMII_10G_KR;
			else if (qlm == 5 || qlm == 6)
				mode[qlm] = CVMX_QLM_MODE_10G_KR_1X2;
			else
				mode[qlm] = CVMX_QLM_MODE_10G_KR;
			ref_clock_sel[qlm] = 2;
			if (qlm == 5 || qlm == 6)
				ref_clock_input[qlm] = 2; // use QLMC_REF_CLK1
			printf("QLM %d: 10G_KR\n", qlm);
		} else if (!strncmp(mode_str, "40G_KR4", 7)) {
			speed[qlm] = 103125;
			mode[qlm] = CVMX_QLM_MODE_40G_KR4;
			ref_clock_sel[qlm] = 2;
			if (qlm == 5 || qlm == 6)
				ref_clock_input[qlm] = 2; // use QLMC_REF_CLK1
			printf("QLM %d: 40G_KR4\n", qlm);
		} else if (!strcmp(mode_str, "pcie")) {
			char *pmode;
			int lanes = 0;
			sprintf(env_var, "pcie%d_mode", qlm);
			pmode = getenv(env_var);
			if (pmode && !strcmp(pmode, "ep"))
				pcie_rc[qlm] = 0;
			else
				pcie_rc[qlm] = 1;
			sprintf(env_var, "pcie%d_gen", qlm);
			pcie_gen[qlm] = getenv_ulong(env_var, 0, 3);
			sprintf(env_var, "pcie%d_lanes", qlm);
			lanes = getenv_ulong(env_var, 0, 8);
			if (lanes == 8)
				mode[qlm] = CVMX_QLM_MODE_PCIE_1X8;
			else if (qlm == 5 || qlm == 6) {
				if (lanes != 2)
					printf("QLM%d: Invalid lanes selected, defaulting to 2 lanes\n", qlm);
				mode[qlm] = CVMX_QLM_MODE_PCIE_1X2;
				ref_clock_input[qlm] = 1; // use QLMC_REF_CLK0
			} else
				mode[qlm] = CVMX_QLM_MODE_PCIE;
			ref_clock_sel[qlm] = 0;
			printf("QLM %d: PCIe gen%d %s, x%d lanes\n", qlm, pcie_gen[qlm] + 1,
				pcie_rc[qlm] ? "root complex" : "endpoint", lanes);
		} else if (!strcmp(mode_str, "sata")) {
			mode[qlm] = CVMX_QLM_MODE_SATA_2X1;
			ref_clock_sel[qlm] = 0;
			ref_clock_input[qlm] = 1;
			speed[qlm] = 6000;
		} else {
			printf("QLM %d: disabled\n", qlm);
		}
	}

	for (qlm = 0; qlm < 7; qlm++) {
		int rc;
		if (mode[qlm] == -1)
			continue;
		debug("Configuring qlm%d with speed(%d), mode(%d), RC(%d), Gen(%d), REF_CLK(%d), CLK_SOURCE(%d)\n",
		      qlm, speed[qlm], mode[qlm], pcie_rc[qlm],
		      pcie_gen[qlm] + 1, ref_clock_sel[qlm], ref_clock_input[qlm]);
		rc = octeon_configure_qlm(qlm, speed[qlm], mode[qlm],
					  pcie_rc[qlm], pcie_gen[qlm],
					  ref_clock_sel[qlm],
					  ref_clock_input[qlm]);

		if (speed[qlm] == 6250)
			octeon_qlm_tune_v3(0, qlm, speed[qlm], 0xa, 0xa0, -1, -1);
		else if (speed[qlm] == 103125)
			octeon_qlm_tune_v3(0, qlm, speed[qlm], 0xd, 0xd0, -1, -1);

		if (qlm == 4 && rc != 0)
			/* There is a bug with SATA with 73xx.  Until it's fixed
			 * we need to strip it from the device tree.
			 */
			octeon_fdt_patch_rename((void *)gd->fdt_blob, "4,none",
						NULL, true, NULL, NULL);
	}

	gpio_set_value(27, 0);	/* Put RGMII PHY in reset */
	mdelay(10);
	gpio_set_value(27, 1);	/* Take RGMII PHY out of reset */
}

int checkboard(void)
{
#if 0 // NO MCU installed so this part can be ignored
	checkboardinfo();
#endif
	return 0;
}

static void *translate_flash_addr(void *base)
{
	void *ptr = base;

	if (!(gd->flags & GD_FLG_RAM_RESIDENT)) {
		cvmx_mio_boot_reg_cfgx_t __attribute__((unused)) reg_cfg;
		uint32_t normal_base = ((CONFIG_SYS_FLASH_BASE >> 16)
		                        & 0x1fff);
		uint32_t offset;

		reg_cfg.u64 = cvmx_read_csr(CVMX_MIO_BOOT_REG_CFG0);
		offset = (normal_base - reg_cfg.s.base) << 16;
		ptr = (void *)((char *) base - offset);
	}

	return ptr;
}

static void *get_ubnt_gd_addr(void)
{
	return translate_flash_addr((void *)CONFIG_UBNT_GD_ADDR);
}

static void *get_ubnt_eeprom_addr(void)
{
	return translate_flash_addr((void *)CONFIG_UBNT_EEPROM_ADDR);
}

#define UBNT_MAX_DESC_LEN 64

static int validate_and_fix_gd_entry(octeon_eeprom_header_t *hdr)
{
	int i;
	uint16_t csum = 0;
	uint8_t *p = (uint8_t *) hdr;

	be16_to_cpus(hdr->type);
	switch (hdr->type) {
	case EEPROM_MAC_ADDR_TYPE:
		break;
	case EEPROM_BOARD_DESC_TYPE:
		{
			octeon_eeprom_board_desc_t *bd
				= (octeon_eeprom_board_desc_t *) hdr;
			be16_to_cpus(bd->board_type);
		}
		break;
	default:
		return 0;
	}

	be16_to_cpus(hdr->length);
	if (hdr->length > UBNT_MAX_DESC_LEN)
		return 0;

	be16_to_cpus(hdr->checksum);
	for (i = 0; i < hdr->length; i++)
		csum += p[i];

	csum -= (hdr->checksum & 0xff);
	csum -= (hdr->checksum >> 8);
	if (csum != hdr->checksum)
		return 0;

	return 1;
}

void print_mpr()
{
	void *eptr = get_ubnt_eeprom_addr();
	uint8_t edata[32];
	uint32_t mpt;

	memcpy(edata, eptr, 32);
	mpt = (edata[16] << 16) + (edata[17] << 8) + edata[18];
	printf("MPR 13-%05u-%02u\n", mpt, edata[19]);
}

int early_board_init(void)
{
	uint8_t value;
	uint8_t addr;
	uint8_t reg;
	uint8_t tempoffset;
	uint8_t num;
	uint8_t ct;
	struct cregs *padt;

	octeon_board_get_clock_info(EBB7304_DEF_DRAM_FREQ);
//	octeon_board_get_descriptor(CVMX_BOARD_TYPE_UBNT_E1000, 1, 0);
	
	/* Even though the CPU ref freq is stored in the clock descriptor, we
	 * don't read it here.  The MCU reads it and configures the clock, and
	 * we read how the clock is actually configured.
	 * The bootloader does not need to read the clock descriptor tuple for
	 * normal operation on rev 2 and later boards.
	 */

//	octeon_board_get_mac_addr();

	{
		void *gdptr = get_ubnt_gd_addr();
		memcpy((void *)&(gd->arch.mac_desc),
			   gdptr + CONFIG_UBNT_GD_MAC_DESC_OFF,
			   sizeof(octeon_eeprom_mac_addr_t));
		memcpy((void *)&(gd->arch.board_desc),
			   gdptr + CONFIG_UBNT_GD_BOARD_DESC_OFF,
			   sizeof(octeon_eeprom_board_desc_t));
		memset((void *)&(gd->arch.clock_desc), 0x0,
			   sizeof(octeon_eeprom_clock_desc_t));

		if (!validate_and_fix_gd_entry(&(gd->arch.mac_desc.header))
			|| gd->arch.mac_desc.header.type != EEPROM_MAC_ADDR_TYPE) {
			gd->arch.mac_desc.count = 8;
			gd->arch.mac_desc.mac_addr_base[0] = 0x00;
			gd->arch.mac_desc.mac_addr_base[1] = 0xbe;
			gd->arch.mac_desc.mac_addr_base[2] = 0xef;
			gd->arch.mac_desc.mac_addr_base[3] = 0x10;
			gd->arch.mac_desc.mac_addr_base[4] = 0x00;
			gd->arch.mac_desc.mac_addr_base[5] = 0x00;
		}   

		if (!validate_and_fix_gd_entry(&(gd->arch.board_desc.header))
			|| gd->arch.board_desc.header.type != EEPROM_BOARD_DESC_TYPE
			|| ((gd->arch.board_desc.board_type != CVMX_BOARD_TYPE_UBNT_E1000)
			&& (gd->arch.board_desc.board_type != CVMX_BOARD_TYPE_UBNT_E1020))) {
			strncpy((char *)(gd->arch.board_desc.serial_str),
					"fffffffffffffffff", SERIAL_LEN);
				gd->arch.board_desc.rev_minor = 0xff;
				gd->arch.board_desc.board_type = CVMX_BOARD_TYPE_UBNT_E1000;
				gd->arch.board_desc.rev_major = BOARD_E1000_MAJOR;
		}
	}

	i2c_set_bus_num(0);
	i2c_probe(PCA9548_I2CADDR);
	i2c_reg_write(PCA9548_I2CADDR, 0, PCA9548_CHAN1);
	mdelay(10);
	i2c_probe(ADT7475_I2CADDR);

	addr = ADT7475_I2CADDR;
	reg = CONFIG5_REG;
	value = i2c_reg_read(addr, reg);
	if ((value & BIT0) == 0) {
		tempoffset = 64;
	} else {
		tempoffset = 0;
	}

	padt = &adt7475[0];
	num = sizeof(adt7475)/sizeof(struct cregs);
	for (ct = 0; ct < num; ct++) {
		i2c_reg_write(ADT7475_I2CADDR, padt->addr, padt->vl);
#if defined(DEBUG)
		mdelay(10);
		value = i2c_reg_read(addr, padt->addr);
		printf("reg: 0x%X, value: 0x%X\n", padt->addr, value);
#endif
		padt++;
	}

#if defined(DEBUG)
	reg = REMOTE1_TEMP_READ_REG;
	value = i2c_reg_read(addr, reg);
	value -= tempoffset;
	printf("Configuring the ADT7475 to automatic mode ...\n");
	printf("Current CPU temperature: %d\n", value);
#endif

	i2c_reg_write(PCA9548_I2CADDR, PCA9548_CONFIG_REG, PCA9548_CHAN0|PCA9548_CHAN1);   // Close all channel of I2C

	return 0;
}

int late_board_init(void)
{
	uint8_t val;
	uint8_t addr;
	uint8_t reg;
	uint16_t tachl;
	uint16_t tachh;
	uint16_t speed;

//  This is a workaround to resolve the i2c hanging off before going to linux shell
	i2c_probe(0x71);
//  Configure PCA9555 0x20
	i2c_probe(0x20);
//  Configure SFP1-8_MOD_ABS,SFP1-8_OPRX_LOS as input
	i2c_reg_write(0x20, 0x6, 0xff);
	i2c_reg_write(0x20, 0x7, 0xff);
//	Configure PCA9555 0x21
	i2c_probe(0x21);
//	Configure SFP1-8_OPTX_DIS (PortA-0x06) as output ,SFP1-8_OPTX_FLT as input
	i2c_reg_write(0x21, 0x6, 0x00);
	i2c_reg_write(0x21, 0x7, 0xff);
//  Set SFP1-8_OPTX_DIS as 0	
	i2c_reg_write(0x21, 0x2, 0x00);
	mdelay(10);

#if defined(DEBUG)
	i2c_probe(PCA9548_I2CADDR);
	i2c_reg_write(PCA9548_I2CADDR, 0, PCA9548_CHAN1);
	mdelay(10);
	i2c_probe(ADT7475_I2CADDR);

	addr = ADT7475_I2CADDR;
	reg = TACH1_LOW_REG;
	tachl = i2c_reg_read(addr, reg);

	addr = ADT7475_I2CADDR;
	reg = TACH1_HIGH_REG;
	tachh = i2c_reg_read(addr, reg);

	speed = (TACH_OSC_FREQ*MIN2SEC)/((tachh << 8)|tachl);
	printf("U-boot phase: current fan speed: %d rpm\n", speed);

	addr = ADT7475_I2CADDR;
	reg = PWM1_CUR_DUTY_REG;
	val = i2c_reg_read(addr, reg);
	val = (val*PWM2STEP)/100;
	printf("U-boot phase: current PWM duty cycle: %d percentage\n", val);
#endif

	return 0;
}

/****** ubntw ******/

static void fill_checksum(octeon_eeprom_header_t *hdr)
{
	int i;
	uint16_t csum = 0;
	uint8_t *p = (uint8_t *) hdr;
	unsigned int len = ntohs(hdr->length);

	for (i = 0; i < len; i++)
		csum += p[i];

	hdr->checksum = htons(csum);
}

static int ubnt_write_flash(void *dst, void *src, int size)
{
    ulong offset, blk_size, off_in_blk, blk_base, blk_head_addr;
    int r, ret;
    void *buf;

    ret = 0;
    offset = ((unsigned int)dst) - CONFIG_SYS_FLASH_BASE;
    if ((offset < 0) || (offset > CONFIG_SYS_FLASH_SIZE)) {
        return 1;
    }


    if ((CONFIG_SYS_FLASH_SIZE - offset) <= (64 * 1024)) {
        blk_size = 8 * 1024;
        blk_base = CONFIG_SYS_FLASH_BASE + CONFIG_SYS_FLASH_SIZE - (64 * 1024);
    } else {
        blk_size = 64 * 1024;
        blk_base = CONFIG_SYS_FLASH_BASE;
    }

	offset = ((ulong)dst) - blk_base;
    off_in_blk = offset & (blk_size - 1);
    if ((off_in_blk + size) > blk_size) {
        /* FIXME: multiple blocks support */
        return 1;
    }

    if (!(buf = (void *) malloc(blk_size))) {
        printf("%s: Failed to allocate memory(%lu)\n", __FUNCTION__, blk_size);
        return 1;
    }

    blk_head_addr = blk_base + (offset - off_in_blk);

    memcpy(buf, (void *)blk_head_addr, blk_size);
    memcpy(buf + off_in_blk, src, size);

    if ((r = flash_sect_protect(0, blk_head_addr, blk_head_addr + blk_size - 1))) {
        printf("%s: Failed on flash_sect_protect from:0x%08lx, len:%lu\n", __FUNCTION__, blk_head_addr, blk_size);
        flash_perror(r);
        ret = 1;
        goto out;
    }
    if ((r = flash_sect_erase(blk_head_addr, blk_head_addr + blk_size - 1))) {
        printf("%s: Failed on flash_sect_erase from:0x%08lx, len:%lu\n", __FUNCTION__, blk_head_addr, blk_size);
        flash_perror(r);
        ret = 1;
        goto out;
    }
    if ((r = flash_write(buf, blk_head_addr, blk_size))) {
        printf("%s: Failed on flash_sect_write from:0x%08lx, len:%lu\n", __FUNCTION__, blk_head_addr, blk_size);
        flash_perror(r);
        ret = 1;
        goto out;
    }

out:
    free(buf);
    printf("\n");
    return ret;
}

static int read_ubnt_gd_entries(octeon_eeprom_header_t **hdr)
{
	int i, r, ret = 0;
	unsigned int gdoff;
	void *gdptr = get_ubnt_gd_addr();
	void *buf;

	if (!hdr) {
		printf("Invalid GD entry\n");
		return 1;
	}

	if (!(buf = (void *) malloc(CONFIG_UBNT_GD_SIZE))) {
		printf("Failed to allocate memory\n");
		return 1;
	}

	memcpy(buf, gdptr, CONFIG_UBNT_GD_SIZE);
	for (i = 0; hdr[i]; i++) {
		unsigned int len = ntohs(hdr[i]->length);

		fill_checksum(hdr[i]);
		switch (ntohs(hdr[i]->type)) {
		case EEPROM_MAC_ADDR_TYPE:
			gdoff = CONFIG_UBNT_GD_MAC_DESC_OFF;
			break;
		case EEPROM_BOARD_DESC_TYPE:
			gdoff = CONFIG_UBNT_GD_BOARD_DESC_OFF;
			break;
		default:
			printf("Unsupported type\n");
			ret = 1;
			goto out;
		}

		memcpy(hdr[i], buf + gdoff, len);
	}

out:
	free(buf);
	return ret;
}

static int write_ubnt_gd_entries(octeon_eeprom_header_t **hdr)
{
	int i, r, ret = 0;
	unsigned int gdoff;
	void *gdptr = get_ubnt_gd_addr();
	void *buf;

	if (!hdr) {
		printf("Invalid GD entry\n");
		return 1;
	}

	if (!(buf = (void *) malloc(CONFIG_UBNT_GD_SIZE))) {
		printf("Failed to allocate memory\n");
		return 1;
	}

	memcpy(buf, gdptr, CONFIG_UBNT_GD_SIZE);
	for (i = 0; hdr[i]; i++) {
		unsigned int len = ntohs(hdr[i]->length);

		fill_checksum(hdr[i]);
		switch (ntohs(hdr[i]->type)) {
		case EEPROM_MAC_ADDR_TYPE:
			gdoff = CONFIG_UBNT_GD_MAC_DESC_OFF;
			break;
		case EEPROM_BOARD_DESC_TYPE:
			gdoff = CONFIG_UBNT_GD_BOARD_DESC_OFF;
			break;
		default:
			printf("Unsupported type\n");
			ret = 1;
			goto out;
		}

		memcpy(buf + gdoff, hdr[i], len);
	}

	printf("write_ubnt_gd_entries gdptr 0x%08x\n", gdptr);

    if ((r = ubnt_write_flash(gdptr, buf, CONFIG_UBNT_GD_SIZE))) {
        ret = 1; 
        goto out;
    }

out:
	free(buf);
	printf("\n");
	return ret;
}

struct board_type_info {
	const char *type;
    uint16_t board_type;
	uint8_t major;
	uint8_t dr_id[4];
};


static struct board_type_info board_types[] = {
	{"e1000", CVMX_BOARD_TYPE_UBNT_E1000, BOARD_E1000_MAJOR, {0xee, 0x51, 0x07, 0x77}},
	{"e1020", CVMX_BOARD_TYPE_UBNT_E1020, BOARD_E1020_MAJOR, {0xee, 0x33, 0x07, 0x77}},
	{NULL, 0xff, 0xff, {0xff, 0xff, 0xff, 0xff}}
};

static struct board_type_info *find_board_type(const char *btype)
{
	int i;

	for (i = 0; board_types[i].type; i++)
		if (strcmp(btype, board_types[i].type) == 0)
			return &(board_types[i]);

	return NULL;
}

static int write_ubnt_eeprom(struct board_type_info *binfo,
			     octeon_eeprom_mac_addr_t *mac,
			     uint8_t *mpr)
{
	int i, r;
	uint8_t data[32];
	uint8_t *d = binfo->dr_id;
	void *eptr = get_ubnt_eeprom_addr();

	memset(data, 0, 32);
	for (i = 0; i < 6; i++) {
		data[i] = mac->mac_addr_base[i];
		data[i + 6] = mac->mac_addr_base[i];
	}
	data[6] |= 0x02;
	for (i = 0; i < 4; i++) {
		data[(12 + i)] = d[i];
		data[(16 + i)] = mpr[i];
	}

	printf("write_ubnt_eeprom 0x%08x\n", eptr);
	if ((r = ubnt_write_flash(eptr, data, 32))) {
		return 1;
	}
	return 0;
}

static void fill_mac_desc(octeon_eeprom_mac_addr_t *mac, const char *mstr,
			  const char *mcount)
{
	int i;
	uint64_t maddr;

	mac->header.type = htons(EEPROM_MAC_ADDR_TYPE);
	mac->header.length = htons(sizeof(octeon_eeprom_mac_addr_t));
	mac->header.minor_version = 0;
	mac->header.major_version = 1;
	mac->header.checksum = 0;

	maddr = simple_strtoull(mstr, NULL, 16);
	cpu_to_be64s(maddr);
	mac->count = simple_strtoul(mcount, NULL, 10);
	for (i = 0; i < 6; i++) {
		mac->mac_addr_base[i] = (maddr >> ((5 - i) * 8)) & 0xff;
	}
}

static void fill_board_desc(octeon_eeprom_board_desc_t *bd,
			    struct board_type_info *binfo,
			    const char *b_minor, const char *serial)
{
	bd->header.type = htons(EEPROM_BOARD_DESC_TYPE);
	bd->header.length = htons(sizeof(octeon_eeprom_board_desc_t));
	bd->header.minor_version = 0;
	bd->header.major_version = 1;
	bd->header.checksum = 0;

	bd->board_type = htons(binfo->board_type);
	bd->rev_major = binfo->major;
	bd->rev_minor = simple_strtoul(b_minor, NULL, 10);
	strncpy((char *) (bd->serial_str), serial, SERIAL_LEN);
	(bd->serial_str)[SERIAL_LEN - 1] = 0;
}

static void show_mac_desc(octeon_eeprom_mac_addr_t *mac)
{
	if (!mac) {
		return;
	}

	printf("MAC base: %02x:%02x:%02x:%02x:%02x:%02x, count: %u\n",
	       mac->mac_addr_base[0], mac->mac_addr_base[1],
	       mac->mac_addr_base[2], mac->mac_addr_base[3],
	       mac->mac_addr_base[4], mac->mac_addr_base[5], mac->count);
}

static void show_board_desc(octeon_eeprom_board_desc_t *bd)
{
	if (!bd) {
		return;
	}

	printf("Board type: %s (%u.%u)\nSerial number: %s\n",
	       cvmx_board_type_to_string(ntohs(bd->board_type)),
	       bd->rev_major, bd->rev_minor, bd->serial_str);
}

int do_ubntw(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	char op;

	if (!argv[1]) {
		printf("Operation required\n");
		return 1;
	}

	op = argv[1][0];
	if (op == 'a') {
		octeon_eeprom_mac_addr_t mac;
		octeon_eeprom_board_desc_t bd;
		octeon_eeprom_header_t *hdrs[] = {
			((octeon_eeprom_header_t *) &mac),
			((octeon_eeprom_header_t *) &bd),
			NULL
		};
		struct board_type_info *binfo;
		uint8_t mpr[4] = {0xff, 0xff, 0xff, 0xff};

		if (argc < 7) {
			printf("Required arguments missing\n");
			return 1;
		}

		if (!(binfo = find_board_type(argv[4]))) {
			printf("Invalid type\n");
			return 1;
		}

		fill_mac_desc(&mac, argv[2], argv[3]);
		fill_board_desc(&bd, binfo, argv[5], argv[6]);

		mpr[3] = simple_strtoul(argv[5], NULL, 10);
		if (argc == 8) {
			uint32_t pt = simple_strtoul(argv[7], NULL, 10);
			mpr[0] = (pt >> 16) & 0xff;
			mpr[1] = (pt >> 8) & 0xff;
			mpr[2] = pt & 0xff;
		}

		if (write_ubnt_gd_entries(hdrs) != 0) {
			printf("Write data failed\n");
			return 1;
		}
		if (write_ubnt_eeprom(binfo, &mac, mpr) != 0) {
			printf("Write failed\n");
			return 1;
		}

		show_mac_desc(&mac);
		show_board_desc(&bd);
		return 0;
    } else if (op == 'd') {                                                        
		struct board_type_info *binfo = &board_types[0];
		octeon_eeprom_mac_addr_t mac;
		octeon_eeprom_board_desc_t bd;
		octeon_eeprom_header_t *hdrs[] = {
			((octeon_eeprom_header_t *) &mac),
			((octeon_eeprom_header_t *) &bd),
			NULL
		};
                                                                                   
		fill_mac_desc(&mac, "000000000000", "0");
		fill_board_desc(&bd, binfo, "0", "");
		if (read_ubnt_gd_entries(hdrs) != 0) {
			printf("Read data failed\n");
			return 1;
		}
		show_mac_desc(&mac);
		show_board_desc(&bd);
		return 0;
    }

	printf("Invalid operation\n");
	return 1;
}

U_BOOT_CMD(ubntw, 8, 1, do_ubntw,
	"ubntw      - ubntw command\n",
	"    all <mac> <count> <type> <minor> <serial> [<pt>] - write all\n"
    "    dump - dump ubntw info");

static void _set_led(int l)
{
	int mbus = 0;
	int maddr = 1;
	uint16_t val = 0;
	switch (l) {
	case 0:
		val = 0x300f;
		break;
	case 1:
		val = 0x300a;
		break;
	case 2:
		val = 0x3000;
		break;
	default:
		break;
	}
	cvmx_mdio_write(mbus, maddr, 0x19, val);
}

void board_blink_led(int ms)
{
	int i;
	for (i = 0; i < (ms / 125); i++) {
		_set_led(i % 2);
		udelay(125000);
	}
}

void board_set_led_on()
{
	_set_led(0);
}

void board_set_led_off()
{
	_set_led(1);
}

void board_set_led_normal()
{
	_set_led(2);
}

