// ======================================================================
// Design01.v generated from TopDesign.cysch
// 09/05/2015 at 17:07
// This file is auto generated. ANY EDITS YOU MAKE MAY BE LOST WHEN THIS FILE IS REGENERATED!!!
// ======================================================================

/* -- WARNING: The following section of defines are deprecated and will be removed in a future release -- */
`define CYDEV_CHIP_DIE_LEOPARD 1
`define CYDEV_CHIP_REV_LEOPARD_PRODUCTION 3
`define CYDEV_CHIP_REV_LEOPARD_ES3 3
`define CYDEV_CHIP_REV_LEOPARD_ES2 1
`define CYDEV_CHIP_REV_LEOPARD_ES1 0
`define CYDEV_CHIP_DIE_GEN4 2
`define CYDEV_CHIP_REV_GEN4_PRODUCTION 17
`define CYDEV_CHIP_REV_GEN4_ES 17
`define CYDEV_CHIP_REV_GEN4_ES2 33
`define CYDEV_CHIP_DIE_PSOC4A 3
`define CYDEV_CHIP_REV_PSOC4A_PRODUCTION 17
`define CYDEV_CHIP_REV_PSOC4A_ES0 17
`define CYDEV_CHIP_DIE_PSOC5LP 4
`define CYDEV_CHIP_REV_PSOC5LP_PRODUCTION 0
`define CYDEV_CHIP_REV_PSOC5LP_ES0 0
`define CYDEV_CHIP_DIE_PANTHER 5
`define CYDEV_CHIP_REV_PANTHER_PRODUCTION 1
`define CYDEV_CHIP_REV_PANTHER_ES1 1
`define CYDEV_CHIP_REV_PANTHER_ES0 0
`define CYDEV_CHIP_DIE_EXPECT 4
`define CYDEV_CHIP_REV_EXPECT 0
`define CYDEV_CHIP_DIE_ACTUAL 4
/* -- WARNING: The previous section of defines are deprecated and will be removed in a future release -- */
`define CYDEV_CHIP_FAMILY_UNKNOWN 0
`define CYDEV_CHIP_MEMBER_UNKNOWN 0
`define CYDEV_CHIP_FAMILY_PSOC3 1
`define CYDEV_CHIP_MEMBER_3A 1
`define CYDEV_CHIP_REVISION_3A_PRODUCTION 3
`define CYDEV_CHIP_REVISION_3A_ES3 3
`define CYDEV_CHIP_REVISION_3A_ES2 1
`define CYDEV_CHIP_REVISION_3A_ES1 0
`define CYDEV_CHIP_FAMILY_PSOC4 2
`define CYDEV_CHIP_MEMBER_4G 2
`define CYDEV_CHIP_REVISION_4G_PRODUCTION 17
`define CYDEV_CHIP_REVISION_4G_ES 17
`define CYDEV_CHIP_REVISION_4G_ES2 33
`define CYDEV_CHIP_MEMBER_4D 3
`define CYDEV_CHIP_REVISION_4D_PRODUCTION 0
`define CYDEV_CHIP_MEMBER_4H 4
`define CYDEV_CHIP_REVISION_4H_PRODUCTION 0
`define CYDEV_CHIP_MEMBER_4E 5
`define CYDEV_CHIP_REVISION_4E_PRODUCTION 0
`define CYDEV_CHIP_MEMBER_4A 6
`define CYDEV_CHIP_REVISION_4A_PRODUCTION 17
`define CYDEV_CHIP_REVISION_4A_ES0 17
`define CYDEV_CHIP_MEMBER_4F 7
`define CYDEV_CHIP_REVISION_4F_PRODUCTION 0
`define CYDEV_CHIP_REVISION_4F_PRODUCTION_256K 0
`define CYDEV_CHIP_MEMBER_4F 8
`define CYDEV_CHIP_REVISION_4F_PRODUCTION 0
`define CYDEV_CHIP_REVISION_4F_PRODUCTION_256K 0
`define CYDEV_CHIP_MEMBER_4M 9
`define CYDEV_CHIP_REVISION_4M_PRODUCTION 0
`define CYDEV_CHIP_MEMBER_4L 10
`define CYDEV_CHIP_REVISION_4L_PRODUCTION 0
`define CYDEV_CHIP_MEMBER_4C 11
`define CYDEV_CHIP_REVISION_4C_PRODUCTION 0
`define CYDEV_CHIP_FAMILY_PSOC5 3
`define CYDEV_CHIP_MEMBER_5B 12
`define CYDEV_CHIP_REVISION_5B_PRODUCTION 0
`define CYDEV_CHIP_REVISION_5B_ES0 0
`define CYDEV_CHIP_MEMBER_5A 13
`define CYDEV_CHIP_REVISION_5A_PRODUCTION 1
`define CYDEV_CHIP_REVISION_5A_ES1 1
`define CYDEV_CHIP_REVISION_5A_ES0 0
`define CYDEV_CHIP_FAMILY_USED 3
`define CYDEV_CHIP_MEMBER_USED 12
`define CYDEV_CHIP_REVISION_USED 0
// Component: STC_Datapath16_v1_0
`ifdef CY_BLK_DIR
`undef CY_BLK_DIR
`endif

`ifdef WARP
`define CY_BLK_DIR "..\STC_Datapath16_v1_0"
`include "..\STC_Datapath16_v1_0\STC_Datapath16_v1_0.v"
`else
`define CY_BLK_DIR "Z:\scottcarlson On My Mac\Code\PSoC_Creator\Design01\Design01.cydsn\Design01.cydsn\STC_Datapath16_v1_0"
`include "Z:\scottcarlson On My Mac\Code\PSoC_Creator\Design01\Design01.cydsn\Design01.cydsn\STC_Datapath16_v1_0\STC_Datapath16_v1_0.v"
`endif

// Component: CyStatusReg_v1_90
`ifdef CY_BLK_DIR
`undef CY_BLK_DIR
`endif

`ifdef WARP
`define CY_BLK_DIR "$CYPRESS_DIR\..\psoc\content\cyprimitives\CyPrimitives.cylib\CyStatusReg_v1_90"
`include "$CYPRESS_DIR\..\psoc\content\cyprimitives\CyPrimitives.cylib\CyStatusReg_v1_90\CyStatusReg_v1_90.v"
`else
`define CY_BLK_DIR "C:\Program Files\Cypress\PSoC Creator\3.2\PSoC Creator\psoc\content\cyprimitives\CyPrimitives.cylib\CyStatusReg_v1_90"
`include "C:\Program Files\Cypress\PSoC Creator\3.2\PSoC Creator\psoc\content\cyprimitives\CyPrimitives.cylib\CyStatusReg_v1_90\CyStatusReg_v1_90.v"
`endif

// top
module top ;

          wire [7:0] Net_57;
          wire  Net_56;
          wire  Net_55;
          wire  Net_54;
          wire  Net_53;
          wire  Net_52;
          wire  Net_50;
          wire  Net_45;
          wire  Net_44;
          wire  Net_43;
          wire  Net_42;
          wire  Net_49;
          wire  Net_48;
          wire  Net_47;
          wire  Net_46;
          wire  Net_36;

    STC_Datapath16_v1_0 STC_Datapath16_v1_00 (
        .Clock(Net_36),
        .Reset(1'b0),
        .State_0(Net_47),
        .State_1(Net_48),
        .State_2(Net_49));
    defparam STC_Datapath16_v1_00.A0_init_a = 0;
    defparam STC_Datapath16_v1_00.A0_init_b = 0;
    defparam STC_Datapath16_v1_00.A1_init_a = 0;
    defparam STC_Datapath16_v1_00.A1_init_b = 0;
    defparam STC_Datapath16_v1_00.D0_init_a = 0;
    defparam STC_Datapath16_v1_00.D0_init_b = 0;
    defparam STC_Datapath16_v1_00.D1_init_a = 0;
    defparam STC_Datapath16_v1_00.D1_init_b = 0;


	cy_clock_v1_0
		#(.id("b4b5e17f-634b-4c3d-aaa4-f65d9e49df2e"),
		  .source_clock_id(""),
		  .divisor(0),
		  .period("83333333.3333333"),
		  .is_direct(0),
		  .is_digital(1))
		Clock_1
		 (.clock_out(Net_36));



	cy_dma_v1_0
		#(.drq_type(2'b10))
		DMA_1
		 (.drq(1'b0),
		  .trq(1'b0),
		  .nrq(Net_45));


    CyStatusReg_v1_90 Status_Reg_1 (
        .status_0(Net_47),
        .status_1(Net_48),
        .status_2(Net_49),
        .status_3(1'b0),
        .clock(Net_36),
        .status_4(1'b0),
        .status_5(1'b0),
        .status_6(1'b0),
        .status_7(1'b0),
        .intr(Net_56),
        .status_bus(8'b0));
    defparam Status_Reg_1.Bit0Mode = 0;
    defparam Status_Reg_1.Bit1Mode = 0;
    defparam Status_Reg_1.Bit2Mode = 0;
    defparam Status_Reg_1.Bit3Mode = 0;
    defparam Status_Reg_1.Bit4Mode = 0;
    defparam Status_Reg_1.Bit5Mode = 0;
    defparam Status_Reg_1.Bit6Mode = 0;
    defparam Status_Reg_1.Bit7Mode = 0;
    defparam Status_Reg_1.BusDisplay = 0;
    defparam Status_Reg_1.Interrupt = 0;
    defparam Status_Reg_1.MaskValue = 0;
    defparam Status_Reg_1.NumInputs = 8;



endmodule

