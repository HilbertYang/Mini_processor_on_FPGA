////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 1995-2008 Xilinx, Inc.  All rights reserved.
////////////////////////////////////////////////////////////////////////////////
//   ____  ____ 
//  /   /\/   / 
// /___/  \  /    Vendor: Xilinx 
// \   \   \/     Version : 10.1
//  \   \         Application : sch2verilog
//  /   /         Filename : register_file.vf
// /___/   /\     Timestamp : 02/10/2026 09:02:48
// \   \  /  \ 
//  \___\/\___\ 
//
//Command: C:\Xilinx\10.1\ISE\bin\nt\unwrapped\sch2verilog.exe -intstyle ise -family virtex2p -w "C:/Documents and Settings/student/wtut_sc/simu4t/L5/PIPELINE/register_file.sch" register_file.vf
//Design Name: register_file
//Device: virtex2p
//Purpose:
//    This verilog netlist is translated from an ECS schematic.It can be 
//    synthesized and simulated, but it should not be modified. 
//
`timescale 1ns / 1ps

module D2_4E_MXILINX_register_file(A0, 
                                   A1, 
                                   E, 
                                   D0, 
                                   D1, 
                                   D2, 
                                   D3);

    input A0;
    input A1;
    input E;
   output D0;
   output D1;
   output D2;
   output D3;
   
   
   AND3 I_36_30 (.I0(A1), 
                 .I1(A0), 
                 .I2(E), 
                 .O(D3));
   AND3B1 I_36_31 (.I0(A0), 
                   .I1(A1), 
                   .I2(E), 
                   .O(D2));
   AND3B1 I_36_32 (.I0(A1), 
                   .I1(A0), 
                   .I2(E), 
                   .O(D1));
   AND3B2 I_36_33 (.I0(A0), 
                   .I1(A1), 
                   .I2(E), 
                   .O(D0));
endmodule
`timescale 1ns / 1ps

module register_file(clk, 
                     reset, 
                     r0addr, 
                     r1addr, 
                     waddr0, 
                     waddr1, 
                     wdata, 
                     wen, 
                     r0data, 
                     r1data);

    input clk;
    input reset;
    input [1:0] r0addr;
    input [1:0] r1addr;
    input waddr0;
    input waddr1;
    input [63:0] wdata;
    input wen;
   output [63:0] r0data;
   output [63:0] r1data;
   
   wire XLXN_5;
   wire XLXN_7;
   wire XLXN_8;
   wire XLXN_9;
   wire XLXN_13;
   wire [63:0] XLXN_19;
   wire [63:0] XLXN_22;
   wire [63:0] XLXN_24;
   wire [63:0] XLXN_25;
   wire XLXN_32;
   wire XLXN_33;
   wire XLXN_34;
   wire XLXN_35;
   
   FDCE R0_0 (.C(clk), 
              .CE(XLXN_33), 
              .CLR(reset), 
              .D(wdata[0]), 
              .Q(XLXN_19[0]));
   defparam R0_0.INIT = 1'b0;
   FDCE R0_1 (.C(clk), 
              .CE(XLXN_33), 
              .CLR(reset), 
              .D(wdata[1]), 
              .Q(XLXN_19[1]));
   defparam R0_1.INIT = 1'b0;
   FDCE R0_2 (.C(clk), 
              .CE(XLXN_33), 
              .CLR(reset), 
              .D(wdata[2]), 
              .Q(XLXN_19[2]));
   defparam R0_2.INIT = 1'b0;
   FDCE R0_3 (.C(clk), 
              .CE(XLXN_33), 
              .CLR(reset), 
              .D(wdata[3]), 
              .Q(XLXN_19[3]));
   defparam R0_3.INIT = 1'b0;
   FDCE R0_4 (.C(clk), 
              .CE(XLXN_33), 
              .CLR(reset), 
              .D(wdata[4]), 
              .Q(XLXN_19[4]));
   defparam R0_4.INIT = 1'b0;
   FDCE R0_5 (.C(clk), 
              .CE(XLXN_33), 
              .CLR(reset), 
              .D(wdata[5]), 
              .Q(XLXN_19[5]));
   defparam R0_5.INIT = 1'b0;
   FDCE R0_6 (.C(clk), 
              .CE(XLXN_33), 
              .CLR(reset), 
              .D(wdata[6]), 
              .Q(XLXN_19[6]));
   defparam R0_6.INIT = 1'b0;
   FDCE R0_7 (.C(clk), 
              .CE(XLXN_33), 
              .CLR(reset), 
              .D(wdata[7]), 
              .Q(XLXN_19[7]));
   defparam R0_7.INIT = 1'b0;
   FDCE R0_8 (.C(clk), 
              .CE(XLXN_33), 
              .CLR(reset), 
              .D(wdata[8]), 
              .Q(XLXN_19[8]));
   defparam R0_8.INIT = 1'b0;
   FDCE R0_9 (.C(clk), 
              .CE(XLXN_33), 
              .CLR(reset), 
              .D(wdata[9]), 
              .Q(XLXN_19[9]));
   defparam R0_9.INIT = 1'b0;
   FDCE R0_10 (.C(clk), 
               .CE(XLXN_33), 
               .CLR(reset), 
               .D(wdata[10]), 
               .Q(XLXN_19[10]));
   defparam R0_10.INIT = 1'b0;
   FDCE R0_11 (.C(clk), 
               .CE(XLXN_33), 
               .CLR(reset), 
               .D(wdata[11]), 
               .Q(XLXN_19[11]));
   defparam R0_11.INIT = 1'b0;
   FDCE R0_12 (.C(clk), 
               .CE(XLXN_33), 
               .CLR(reset), 
               .D(wdata[12]), 
               .Q(XLXN_19[12]));
   defparam R0_12.INIT = 1'b0;
   FDCE R0_13 (.C(clk), 
               .CE(XLXN_33), 
               .CLR(reset), 
               .D(wdata[13]), 
               .Q(XLXN_19[13]));
   defparam R0_13.INIT = 1'b0;
   FDCE R0_14 (.C(clk), 
               .CE(XLXN_33), 
               .CLR(reset), 
               .D(wdata[14]), 
               .Q(XLXN_19[14]));
   defparam R0_14.INIT = 1'b0;
   FDCE R0_15 (.C(clk), 
               .CE(XLXN_33), 
               .CLR(reset), 
               .D(wdata[15]), 
               .Q(XLXN_19[15]));
   defparam R0_15.INIT = 1'b0;
   FDCE R0_16 (.C(clk), 
               .CE(XLXN_33), 
               .CLR(reset), 
               .D(wdata[16]), 
               .Q(XLXN_19[16]));
   defparam R0_16.INIT = 1'b0;
   FDCE R0_17 (.C(clk), 
               .CE(XLXN_33), 
               .CLR(reset), 
               .D(wdata[17]), 
               .Q(XLXN_19[17]));
   defparam R0_17.INIT = 1'b0;
   FDCE R0_18 (.C(clk), 
               .CE(XLXN_33), 
               .CLR(reset), 
               .D(wdata[18]), 
               .Q(XLXN_19[18]));
   defparam R0_18.INIT = 1'b0;
   FDCE R0_19 (.C(clk), 
               .CE(XLXN_33), 
               .CLR(reset), 
               .D(wdata[19]), 
               .Q(XLXN_19[19]));
   defparam R0_19.INIT = 1'b0;
   FDCE R0_20 (.C(clk), 
               .CE(XLXN_33), 
               .CLR(reset), 
               .D(wdata[20]), 
               .Q(XLXN_19[20]));
   defparam R0_20.INIT = 1'b0;
   FDCE R0_21 (.C(clk), 
               .CE(XLXN_33), 
               .CLR(reset), 
               .D(wdata[21]), 
               .Q(XLXN_19[21]));
   defparam R0_21.INIT = 1'b0;
   FDCE R0_22 (.C(clk), 
               .CE(XLXN_33), 
               .CLR(reset), 
               .D(wdata[22]), 
               .Q(XLXN_19[22]));
   defparam R0_22.INIT = 1'b0;
   FDCE R0_23 (.C(clk), 
               .CE(XLXN_33), 
               .CLR(reset), 
               .D(wdata[23]), 
               .Q(XLXN_19[23]));
   defparam R0_23.INIT = 1'b0;
   FDCE R0_24 (.C(clk), 
               .CE(XLXN_33), 
               .CLR(reset), 
               .D(wdata[24]), 
               .Q(XLXN_19[24]));
   defparam R0_24.INIT = 1'b0;
   FDCE R0_25 (.C(clk), 
               .CE(XLXN_33), 
               .CLR(reset), 
               .D(wdata[25]), 
               .Q(XLXN_19[25]));
   defparam R0_25.INIT = 1'b0;
   FDCE R0_26 (.C(clk), 
               .CE(XLXN_33), 
               .CLR(reset), 
               .D(wdata[26]), 
               .Q(XLXN_19[26]));
   defparam R0_26.INIT = 1'b0;
   FDCE R0_27 (.C(clk), 
               .CE(XLXN_33), 
               .CLR(reset), 
               .D(wdata[27]), 
               .Q(XLXN_19[27]));
   defparam R0_27.INIT = 1'b0;
   FDCE R0_28 (.C(clk), 
               .CE(XLXN_33), 
               .CLR(reset), 
               .D(wdata[28]), 
               .Q(XLXN_19[28]));
   defparam R0_28.INIT = 1'b0;
   FDCE R0_29 (.C(clk), 
               .CE(XLXN_33), 
               .CLR(reset), 
               .D(wdata[29]), 
               .Q(XLXN_19[29]));
   defparam R0_29.INIT = 1'b0;
   FDCE R0_30 (.C(clk), 
               .CE(XLXN_33), 
               .CLR(reset), 
               .D(wdata[30]), 
               .Q(XLXN_19[30]));
   defparam R0_30.INIT = 1'b0;
   FDCE R0_31 (.C(clk), 
               .CE(XLXN_33), 
               .CLR(reset), 
               .D(wdata[31]), 
               .Q(XLXN_19[31]));
   defparam R0_31.INIT = 1'b0;
   FDCE R0_32 (.C(clk), 
               .CE(XLXN_33), 
               .CLR(reset), 
               .D(wdata[32]), 
               .Q(XLXN_19[32]));
   defparam R0_32.INIT = 1'b0;
   FDCE R0_33 (.C(clk), 
               .CE(XLXN_33), 
               .CLR(reset), 
               .D(wdata[33]), 
               .Q(XLXN_19[33]));
   defparam R0_33.INIT = 1'b0;
   FDCE R0_34 (.C(clk), 
               .CE(XLXN_33), 
               .CLR(reset), 
               .D(wdata[34]), 
               .Q(XLXN_19[34]));
   defparam R0_34.INIT = 1'b0;
   FDCE R0_35 (.C(clk), 
               .CE(XLXN_33), 
               .CLR(reset), 
               .D(wdata[35]), 
               .Q(XLXN_19[35]));
   defparam R0_35.INIT = 1'b0;
   FDCE R0_36 (.C(clk), 
               .CE(XLXN_33), 
               .CLR(reset), 
               .D(wdata[36]), 
               .Q(XLXN_19[36]));
   defparam R0_36.INIT = 1'b0;
   FDCE R0_37 (.C(clk), 
               .CE(XLXN_33), 
               .CLR(reset), 
               .D(wdata[37]), 
               .Q(XLXN_19[37]));
   defparam R0_37.INIT = 1'b0;
   FDCE R0_38 (.C(clk), 
               .CE(XLXN_33), 
               .CLR(reset), 
               .D(wdata[38]), 
               .Q(XLXN_19[38]));
   defparam R0_38.INIT = 1'b0;
   FDCE R0_39 (.C(clk), 
               .CE(XLXN_33), 
               .CLR(reset), 
               .D(wdata[39]), 
               .Q(XLXN_19[39]));
   defparam R0_39.INIT = 1'b0;
   FDCE R0_40 (.C(clk), 
               .CE(XLXN_33), 
               .CLR(reset), 
               .D(wdata[40]), 
               .Q(XLXN_19[40]));
   defparam R0_40.INIT = 1'b0;
   FDCE R0_41 (.C(clk), 
               .CE(XLXN_33), 
               .CLR(reset), 
               .D(wdata[41]), 
               .Q(XLXN_19[41]));
   defparam R0_41.INIT = 1'b0;
   FDCE R0_42 (.C(clk), 
               .CE(XLXN_33), 
               .CLR(reset), 
               .D(wdata[42]), 
               .Q(XLXN_19[42]));
   defparam R0_42.INIT = 1'b0;
   FDCE R0_43 (.C(clk), 
               .CE(XLXN_33), 
               .CLR(reset), 
               .D(wdata[43]), 
               .Q(XLXN_19[43]));
   defparam R0_43.INIT = 1'b0;
   FDCE R0_44 (.C(clk), 
               .CE(XLXN_33), 
               .CLR(reset), 
               .D(wdata[44]), 
               .Q(XLXN_19[44]));
   defparam R0_44.INIT = 1'b0;
   FDCE R0_45 (.C(clk), 
               .CE(XLXN_33), 
               .CLR(reset), 
               .D(wdata[45]), 
               .Q(XLXN_19[45]));
   defparam R0_45.INIT = 1'b0;
   FDCE R0_46 (.C(clk), 
               .CE(XLXN_33), 
               .CLR(reset), 
               .D(wdata[46]), 
               .Q(XLXN_19[46]));
   defparam R0_46.INIT = 1'b0;
   FDCE R0_47 (.C(clk), 
               .CE(XLXN_33), 
               .CLR(reset), 
               .D(wdata[47]), 
               .Q(XLXN_19[47]));
   defparam R0_47.INIT = 1'b0;
   FDCE R0_48 (.C(clk), 
               .CE(XLXN_33), 
               .CLR(reset), 
               .D(wdata[48]), 
               .Q(XLXN_19[48]));
   defparam R0_48.INIT = 1'b0;
   FDCE R0_49 (.C(clk), 
               .CE(XLXN_33), 
               .CLR(reset), 
               .D(wdata[49]), 
               .Q(XLXN_19[49]));
   defparam R0_49.INIT = 1'b0;
   FDCE R0_50 (.C(clk), 
               .CE(XLXN_33), 
               .CLR(reset), 
               .D(wdata[50]), 
               .Q(XLXN_19[50]));
   defparam R0_50.INIT = 1'b0;
   FDCE R0_51 (.C(clk), 
               .CE(XLXN_33), 
               .CLR(reset), 
               .D(wdata[51]), 
               .Q(XLXN_19[51]));
   defparam R0_51.INIT = 1'b0;
   FDCE R0_52 (.C(clk), 
               .CE(XLXN_33), 
               .CLR(reset), 
               .D(wdata[52]), 
               .Q(XLXN_19[52]));
   defparam R0_52.INIT = 1'b0;
   FDCE R0_53 (.C(clk), 
               .CE(XLXN_33), 
               .CLR(reset), 
               .D(wdata[53]), 
               .Q(XLXN_19[53]));
   defparam R0_53.INIT = 1'b0;
   FDCE R0_54 (.C(clk), 
               .CE(XLXN_33), 
               .CLR(reset), 
               .D(wdata[54]), 
               .Q(XLXN_19[54]));
   defparam R0_54.INIT = 1'b0;
   FDCE R0_55 (.C(clk), 
               .CE(XLXN_33), 
               .CLR(reset), 
               .D(wdata[55]), 
               .Q(XLXN_19[55]));
   defparam R0_55.INIT = 1'b0;
   FDCE R0_56 (.C(clk), 
               .CE(XLXN_33), 
               .CLR(reset), 
               .D(wdata[56]), 
               .Q(XLXN_19[56]));
   defparam R0_56.INIT = 1'b0;
   FDCE R0_57 (.C(clk), 
               .CE(XLXN_33), 
               .CLR(reset), 
               .D(wdata[57]), 
               .Q(XLXN_19[57]));
   defparam R0_57.INIT = 1'b0;
   FDCE R0_58 (.C(clk), 
               .CE(XLXN_33), 
               .CLR(reset), 
               .D(wdata[58]), 
               .Q(XLXN_19[58]));
   defparam R0_58.INIT = 1'b0;
   FDCE R0_59 (.C(clk), 
               .CE(XLXN_33), 
               .CLR(reset), 
               .D(wdata[59]), 
               .Q(XLXN_19[59]));
   defparam R0_59.INIT = 1'b0;
   FDCE R0_60 (.C(clk), 
               .CE(XLXN_33), 
               .CLR(reset), 
               .D(wdata[60]), 
               .Q(XLXN_19[60]));
   defparam R0_60.INIT = 1'b0;
   FDCE R0_61 (.C(clk), 
               .CE(XLXN_33), 
               .CLR(reset), 
               .D(wdata[61]), 
               .Q(XLXN_19[61]));
   defparam R0_61.INIT = 1'b0;
   FDCE R0_62 (.C(clk), 
               .CE(XLXN_33), 
               .CLR(reset), 
               .D(wdata[62]), 
               .Q(XLXN_19[62]));
   defparam R0_62.INIT = 1'b0;
   FDCE R0_63 (.C(clk), 
               .CE(XLXN_33), 
               .CLR(reset), 
               .D(wdata[63]), 
               .Q(XLXN_19[63]));
   defparam R0_63.INIT = 1'b0;
   FDCE R1_0 (.C(clk), 
              .CE(XLXN_34), 
              .CLR(reset), 
              .D(wdata[0]), 
              .Q(XLXN_22[0]));
   defparam R1_0.INIT = 1'b0;
   FDCE R1_1 (.C(clk), 
              .CE(XLXN_34), 
              .CLR(reset), 
              .D(wdata[1]), 
              .Q(XLXN_22[1]));
   defparam R1_1.INIT = 1'b0;
   FDCE R1_2 (.C(clk), 
              .CE(XLXN_34), 
              .CLR(reset), 
              .D(wdata[2]), 
              .Q(XLXN_22[2]));
   defparam R1_2.INIT = 1'b0;
   FDCE R1_3 (.C(clk), 
              .CE(XLXN_34), 
              .CLR(reset), 
              .D(wdata[3]), 
              .Q(XLXN_22[3]));
   defparam R1_3.INIT = 1'b0;
   FDCE R1_4 (.C(clk), 
              .CE(XLXN_34), 
              .CLR(reset), 
              .D(wdata[4]), 
              .Q(XLXN_22[4]));
   defparam R1_4.INIT = 1'b0;
   FDCE R1_5 (.C(clk), 
              .CE(XLXN_34), 
              .CLR(reset), 
              .D(wdata[5]), 
              .Q(XLXN_22[5]));
   defparam R1_5.INIT = 1'b0;
   FDCE R1_6 (.C(clk), 
              .CE(XLXN_34), 
              .CLR(reset), 
              .D(wdata[6]), 
              .Q(XLXN_22[6]));
   defparam R1_6.INIT = 1'b0;
   FDCE R1_7 (.C(clk), 
              .CE(XLXN_34), 
              .CLR(reset), 
              .D(wdata[7]), 
              .Q(XLXN_22[7]));
   defparam R1_7.INIT = 1'b0;
   FDCE R1_8 (.C(clk), 
              .CE(XLXN_34), 
              .CLR(reset), 
              .D(wdata[8]), 
              .Q(XLXN_22[8]));
   defparam R1_8.INIT = 1'b0;
   FDCE R1_9 (.C(clk), 
              .CE(XLXN_34), 
              .CLR(reset), 
              .D(wdata[9]), 
              .Q(XLXN_22[9]));
   defparam R1_9.INIT = 1'b0;
   FDCE R1_10 (.C(clk), 
               .CE(XLXN_34), 
               .CLR(reset), 
               .D(wdata[10]), 
               .Q(XLXN_22[10]));
   defparam R1_10.INIT = 1'b0;
   FDCE R1_11 (.C(clk), 
               .CE(XLXN_34), 
               .CLR(reset), 
               .D(wdata[11]), 
               .Q(XLXN_22[11]));
   defparam R1_11.INIT = 1'b0;
   FDCE R1_12 (.C(clk), 
               .CE(XLXN_34), 
               .CLR(reset), 
               .D(wdata[12]), 
               .Q(XLXN_22[12]));
   defparam R1_12.INIT = 1'b0;
   FDCE R1_13 (.C(clk), 
               .CE(XLXN_34), 
               .CLR(reset), 
               .D(wdata[13]), 
               .Q(XLXN_22[13]));
   defparam R1_13.INIT = 1'b0;
   FDCE R1_14 (.C(clk), 
               .CE(XLXN_34), 
               .CLR(reset), 
               .D(wdata[14]), 
               .Q(XLXN_22[14]));
   defparam R1_14.INIT = 1'b0;
   FDCE R1_15 (.C(clk), 
               .CE(XLXN_34), 
               .CLR(reset), 
               .D(wdata[15]), 
               .Q(XLXN_22[15]));
   defparam R1_15.INIT = 1'b0;
   FDCE R1_16 (.C(clk), 
               .CE(XLXN_34), 
               .CLR(reset), 
               .D(wdata[16]), 
               .Q(XLXN_22[16]));
   defparam R1_16.INIT = 1'b0;
   FDCE R1_17 (.C(clk), 
               .CE(XLXN_34), 
               .CLR(reset), 
               .D(wdata[17]), 
               .Q(XLXN_22[17]));
   defparam R1_17.INIT = 1'b0;
   FDCE R1_18 (.C(clk), 
               .CE(XLXN_34), 
               .CLR(reset), 
               .D(wdata[18]), 
               .Q(XLXN_22[18]));
   defparam R1_18.INIT = 1'b0;
   FDCE R1_19 (.C(clk), 
               .CE(XLXN_34), 
               .CLR(reset), 
               .D(wdata[19]), 
               .Q(XLXN_22[19]));
   defparam R1_19.INIT = 1'b0;
   FDCE R1_20 (.C(clk), 
               .CE(XLXN_34), 
               .CLR(reset), 
               .D(wdata[20]), 
               .Q(XLXN_22[20]));
   defparam R1_20.INIT = 1'b0;
   FDCE R1_21 (.C(clk), 
               .CE(XLXN_34), 
               .CLR(reset), 
               .D(wdata[21]), 
               .Q(XLXN_22[21]));
   defparam R1_21.INIT = 1'b0;
   FDCE R1_22 (.C(clk), 
               .CE(XLXN_34), 
               .CLR(reset), 
               .D(wdata[22]), 
               .Q(XLXN_22[22]));
   defparam R1_22.INIT = 1'b0;
   FDCE R1_23 (.C(clk), 
               .CE(XLXN_34), 
               .CLR(reset), 
               .D(wdata[23]), 
               .Q(XLXN_22[23]));
   defparam R1_23.INIT = 1'b0;
   FDCE R1_24 (.C(clk), 
               .CE(XLXN_34), 
               .CLR(reset), 
               .D(wdata[24]), 
               .Q(XLXN_22[24]));
   defparam R1_24.INIT = 1'b0;
   FDCE R1_25 (.C(clk), 
               .CE(XLXN_34), 
               .CLR(reset), 
               .D(wdata[25]), 
               .Q(XLXN_22[25]));
   defparam R1_25.INIT = 1'b0;
   FDCE R1_26 (.C(clk), 
               .CE(XLXN_34), 
               .CLR(reset), 
               .D(wdata[26]), 
               .Q(XLXN_22[26]));
   defparam R1_26.INIT = 1'b0;
   FDCE R1_27 (.C(clk), 
               .CE(XLXN_34), 
               .CLR(reset), 
               .D(wdata[27]), 
               .Q(XLXN_22[27]));
   defparam R1_27.INIT = 1'b0;
   FDCE R1_28 (.C(clk), 
               .CE(XLXN_34), 
               .CLR(reset), 
               .D(wdata[28]), 
               .Q(XLXN_22[28]));
   defparam R1_28.INIT = 1'b0;
   FDCE R1_29 (.C(clk), 
               .CE(XLXN_34), 
               .CLR(reset), 
               .D(wdata[29]), 
               .Q(XLXN_22[29]));
   defparam R1_29.INIT = 1'b0;
   FDCE R1_30 (.C(clk), 
               .CE(XLXN_34), 
               .CLR(reset), 
               .D(wdata[30]), 
               .Q(XLXN_22[30]));
   defparam R1_30.INIT = 1'b0;
   FDCE R1_31 (.C(clk), 
               .CE(XLXN_34), 
               .CLR(reset), 
               .D(wdata[31]), 
               .Q(XLXN_22[31]));
   defparam R1_31.INIT = 1'b0;
   FDCE R1_32 (.C(clk), 
               .CE(XLXN_34), 
               .CLR(reset), 
               .D(wdata[32]), 
               .Q(XLXN_22[32]));
   defparam R1_32.INIT = 1'b0;
   FDCE R1_33 (.C(clk), 
               .CE(XLXN_34), 
               .CLR(reset), 
               .D(wdata[33]), 
               .Q(XLXN_22[33]));
   defparam R1_33.INIT = 1'b0;
   FDCE R1_34 (.C(clk), 
               .CE(XLXN_34), 
               .CLR(reset), 
               .D(wdata[34]), 
               .Q(XLXN_22[34]));
   defparam R1_34.INIT = 1'b0;
   FDCE R1_35 (.C(clk), 
               .CE(XLXN_34), 
               .CLR(reset), 
               .D(wdata[35]), 
               .Q(XLXN_22[35]));
   defparam R1_35.INIT = 1'b0;
   FDCE R1_36 (.C(clk), 
               .CE(XLXN_34), 
               .CLR(reset), 
               .D(wdata[36]), 
               .Q(XLXN_22[36]));
   defparam R1_36.INIT = 1'b0;
   FDCE R1_37 (.C(clk), 
               .CE(XLXN_34), 
               .CLR(reset), 
               .D(wdata[37]), 
               .Q(XLXN_22[37]));
   defparam R1_37.INIT = 1'b0;
   FDCE R1_38 (.C(clk), 
               .CE(XLXN_34), 
               .CLR(reset), 
               .D(wdata[38]), 
               .Q(XLXN_22[38]));
   defparam R1_38.INIT = 1'b0;
   FDCE R1_39 (.C(clk), 
               .CE(XLXN_34), 
               .CLR(reset), 
               .D(wdata[39]), 
               .Q(XLXN_22[39]));
   defparam R1_39.INIT = 1'b0;
   FDCE R1_40 (.C(clk), 
               .CE(XLXN_34), 
               .CLR(reset), 
               .D(wdata[40]), 
               .Q(XLXN_22[40]));
   defparam R1_40.INIT = 1'b0;
   FDCE R1_41 (.C(clk), 
               .CE(XLXN_34), 
               .CLR(reset), 
               .D(wdata[41]), 
               .Q(XLXN_22[41]));
   defparam R1_41.INIT = 1'b0;
   FDCE R1_42 (.C(clk), 
               .CE(XLXN_34), 
               .CLR(reset), 
               .D(wdata[42]), 
               .Q(XLXN_22[42]));
   defparam R1_42.INIT = 1'b0;
   FDCE R1_43 (.C(clk), 
               .CE(XLXN_34), 
               .CLR(reset), 
               .D(wdata[43]), 
               .Q(XLXN_22[43]));
   defparam R1_43.INIT = 1'b0;
   FDCE R1_44 (.C(clk), 
               .CE(XLXN_34), 
               .CLR(reset), 
               .D(wdata[44]), 
               .Q(XLXN_22[44]));
   defparam R1_44.INIT = 1'b0;
   FDCE R1_45 (.C(clk), 
               .CE(XLXN_34), 
               .CLR(reset), 
               .D(wdata[45]), 
               .Q(XLXN_22[45]));
   defparam R1_45.INIT = 1'b0;
   FDCE R1_46 (.C(clk), 
               .CE(XLXN_34), 
               .CLR(reset), 
               .D(wdata[46]), 
               .Q(XLXN_22[46]));
   defparam R1_46.INIT = 1'b0;
   FDCE R1_47 (.C(clk), 
               .CE(XLXN_34), 
               .CLR(reset), 
               .D(wdata[47]), 
               .Q(XLXN_22[47]));
   defparam R1_47.INIT = 1'b0;
   FDCE R1_48 (.C(clk), 
               .CE(XLXN_34), 
               .CLR(reset), 
               .D(wdata[48]), 
               .Q(XLXN_22[48]));
   defparam R1_48.INIT = 1'b0;
   FDCE R1_49 (.C(clk), 
               .CE(XLXN_34), 
               .CLR(reset), 
               .D(wdata[49]), 
               .Q(XLXN_22[49]));
   defparam R1_49.INIT = 1'b0;
   FDCE R1_50 (.C(clk), 
               .CE(XLXN_34), 
               .CLR(reset), 
               .D(wdata[50]), 
               .Q(XLXN_22[50]));
   defparam R1_50.INIT = 1'b0;
   FDCE R1_51 (.C(clk), 
               .CE(XLXN_34), 
               .CLR(reset), 
               .D(wdata[51]), 
               .Q(XLXN_22[51]));
   defparam R1_51.INIT = 1'b0;
   FDCE R1_52 (.C(clk), 
               .CE(XLXN_34), 
               .CLR(reset), 
               .D(wdata[52]), 
               .Q(XLXN_22[52]));
   defparam R1_52.INIT = 1'b0;
   FDCE R1_53 (.C(clk), 
               .CE(XLXN_34), 
               .CLR(reset), 
               .D(wdata[53]), 
               .Q(XLXN_22[53]));
   defparam R1_53.INIT = 1'b0;
   FDCE R1_54 (.C(clk), 
               .CE(XLXN_34), 
               .CLR(reset), 
               .D(wdata[54]), 
               .Q(XLXN_22[54]));
   defparam R1_54.INIT = 1'b0;
   FDCE R1_55 (.C(clk), 
               .CE(XLXN_34), 
               .CLR(reset), 
               .D(wdata[55]), 
               .Q(XLXN_22[55]));
   defparam R1_55.INIT = 1'b0;
   FDCE R1_56 (.C(clk), 
               .CE(XLXN_34), 
               .CLR(reset), 
               .D(wdata[56]), 
               .Q(XLXN_22[56]));
   defparam R1_56.INIT = 1'b0;
   FDCE R1_57 (.C(clk), 
               .CE(XLXN_34), 
               .CLR(reset), 
               .D(wdata[57]), 
               .Q(XLXN_22[57]));
   defparam R1_57.INIT = 1'b0;
   FDCE R1_58 (.C(clk), 
               .CE(XLXN_34), 
               .CLR(reset), 
               .D(wdata[58]), 
               .Q(XLXN_22[58]));
   defparam R1_58.INIT = 1'b0;
   FDCE R1_59 (.C(clk), 
               .CE(XLXN_34), 
               .CLR(reset), 
               .D(wdata[59]), 
               .Q(XLXN_22[59]));
   defparam R1_59.INIT = 1'b0;
   FDCE R1_60 (.C(clk), 
               .CE(XLXN_34), 
               .CLR(reset), 
               .D(wdata[60]), 
               .Q(XLXN_22[60]));
   defparam R1_60.INIT = 1'b0;
   FDCE R1_61 (.C(clk), 
               .CE(XLXN_34), 
               .CLR(reset), 
               .D(wdata[61]), 
               .Q(XLXN_22[61]));
   defparam R1_61.INIT = 1'b0;
   FDCE R1_62 (.C(clk), 
               .CE(XLXN_34), 
               .CLR(reset), 
               .D(wdata[62]), 
               .Q(XLXN_22[62]));
   defparam R1_62.INIT = 1'b0;
   FDCE R1_63 (.C(clk), 
               .CE(XLXN_34), 
               .CLR(reset), 
               .D(wdata[63]), 
               .Q(XLXN_22[63]));
   defparam R1_63.INIT = 1'b0;
   FDCE R2_0 (.C(clk), 
              .CE(XLXN_35), 
              .CLR(reset), 
              .D(wdata[0]), 
              .Q(XLXN_24[0]));
   defparam R2_0.INIT = 1'b0;
   FDCE R2_1 (.C(clk), 
              .CE(XLXN_35), 
              .CLR(reset), 
              .D(wdata[1]), 
              .Q(XLXN_24[1]));
   defparam R2_1.INIT = 1'b0;
   FDCE R2_2 (.C(clk), 
              .CE(XLXN_35), 
              .CLR(reset), 
              .D(wdata[2]), 
              .Q(XLXN_24[2]));
   defparam R2_2.INIT = 1'b0;
   FDCE R2_3 (.C(clk), 
              .CE(XLXN_35), 
              .CLR(reset), 
              .D(wdata[3]), 
              .Q(XLXN_24[3]));
   defparam R2_3.INIT = 1'b0;
   FDCE R2_4 (.C(clk), 
              .CE(XLXN_35), 
              .CLR(reset), 
              .D(wdata[4]), 
              .Q(XLXN_24[4]));
   defparam R2_4.INIT = 1'b0;
   FDCE R2_5 (.C(clk), 
              .CE(XLXN_35), 
              .CLR(reset), 
              .D(wdata[5]), 
              .Q(XLXN_24[5]));
   defparam R2_5.INIT = 1'b0;
   FDCE R2_6 (.C(clk), 
              .CE(XLXN_35), 
              .CLR(reset), 
              .D(wdata[6]), 
              .Q(XLXN_24[6]));
   defparam R2_6.INIT = 1'b0;
   FDCE R2_7 (.C(clk), 
              .CE(XLXN_35), 
              .CLR(reset), 
              .D(wdata[7]), 
              .Q(XLXN_24[7]));
   defparam R2_7.INIT = 1'b0;
   FDCE R2_8 (.C(clk), 
              .CE(XLXN_35), 
              .CLR(reset), 
              .D(wdata[8]), 
              .Q(XLXN_24[8]));
   defparam R2_8.INIT = 1'b0;
   FDCE R2_9 (.C(clk), 
              .CE(XLXN_35), 
              .CLR(reset), 
              .D(wdata[9]), 
              .Q(XLXN_24[9]));
   defparam R2_9.INIT = 1'b0;
   FDCE R2_10 (.C(clk), 
               .CE(XLXN_35), 
               .CLR(reset), 
               .D(wdata[10]), 
               .Q(XLXN_24[10]));
   defparam R2_10.INIT = 1'b0;
   FDCE R2_11 (.C(clk), 
               .CE(XLXN_35), 
               .CLR(reset), 
               .D(wdata[11]), 
               .Q(XLXN_24[11]));
   defparam R2_11.INIT = 1'b0;
   FDCE R2_12 (.C(clk), 
               .CE(XLXN_35), 
               .CLR(reset), 
               .D(wdata[12]), 
               .Q(XLXN_24[12]));
   defparam R2_12.INIT = 1'b0;
   FDCE R2_13 (.C(clk), 
               .CE(XLXN_35), 
               .CLR(reset), 
               .D(wdata[13]), 
               .Q(XLXN_24[13]));
   defparam R2_13.INIT = 1'b0;
   FDCE R2_14 (.C(clk), 
               .CE(XLXN_35), 
               .CLR(reset), 
               .D(wdata[14]), 
               .Q(XLXN_24[14]));
   defparam R2_14.INIT = 1'b0;
   FDCE R2_15 (.C(clk), 
               .CE(XLXN_35), 
               .CLR(reset), 
               .D(wdata[15]), 
               .Q(XLXN_24[15]));
   defparam R2_15.INIT = 1'b0;
   FDCE R2_16 (.C(clk), 
               .CE(XLXN_35), 
               .CLR(reset), 
               .D(wdata[16]), 
               .Q(XLXN_24[16]));
   defparam R2_16.INIT = 1'b0;
   FDCE R2_17 (.C(clk), 
               .CE(XLXN_35), 
               .CLR(reset), 
               .D(wdata[17]), 
               .Q(XLXN_24[17]));
   defparam R2_17.INIT = 1'b0;
   FDCE R2_18 (.C(clk), 
               .CE(XLXN_35), 
               .CLR(reset), 
               .D(wdata[18]), 
               .Q(XLXN_24[18]));
   defparam R2_18.INIT = 1'b0;
   FDCE R2_19 (.C(clk), 
               .CE(XLXN_35), 
               .CLR(reset), 
               .D(wdata[19]), 
               .Q(XLXN_24[19]));
   defparam R2_19.INIT = 1'b0;
   FDCE R2_20 (.C(clk), 
               .CE(XLXN_35), 
               .CLR(reset), 
               .D(wdata[20]), 
               .Q(XLXN_24[20]));
   defparam R2_20.INIT = 1'b0;
   FDCE R2_21 (.C(clk), 
               .CE(XLXN_35), 
               .CLR(reset), 
               .D(wdata[21]), 
               .Q(XLXN_24[21]));
   defparam R2_21.INIT = 1'b0;
   FDCE R2_22 (.C(clk), 
               .CE(XLXN_35), 
               .CLR(reset), 
               .D(wdata[22]), 
               .Q(XLXN_24[22]));
   defparam R2_22.INIT = 1'b0;
   FDCE R2_23 (.C(clk), 
               .CE(XLXN_35), 
               .CLR(reset), 
               .D(wdata[23]), 
               .Q(XLXN_24[23]));
   defparam R2_23.INIT = 1'b0;
   FDCE R2_24 (.C(clk), 
               .CE(XLXN_35), 
               .CLR(reset), 
               .D(wdata[24]), 
               .Q(XLXN_24[24]));
   defparam R2_24.INIT = 1'b0;
   FDCE R2_25 (.C(clk), 
               .CE(XLXN_35), 
               .CLR(reset), 
               .D(wdata[25]), 
               .Q(XLXN_24[25]));
   defparam R2_25.INIT = 1'b0;
   FDCE R2_26 (.C(clk), 
               .CE(XLXN_35), 
               .CLR(reset), 
               .D(wdata[26]), 
               .Q(XLXN_24[26]));
   defparam R2_26.INIT = 1'b0;
   FDCE R2_27 (.C(clk), 
               .CE(XLXN_35), 
               .CLR(reset), 
               .D(wdata[27]), 
               .Q(XLXN_24[27]));
   defparam R2_27.INIT = 1'b0;
   FDCE R2_28 (.C(clk), 
               .CE(XLXN_35), 
               .CLR(reset), 
               .D(wdata[28]), 
               .Q(XLXN_24[28]));
   defparam R2_28.INIT = 1'b0;
   FDCE R2_29 (.C(clk), 
               .CE(XLXN_35), 
               .CLR(reset), 
               .D(wdata[29]), 
               .Q(XLXN_24[29]));
   defparam R2_29.INIT = 1'b0;
   FDCE R2_30 (.C(clk), 
               .CE(XLXN_35), 
               .CLR(reset), 
               .D(wdata[30]), 
               .Q(XLXN_24[30]));
   defparam R2_30.INIT = 1'b0;
   FDCE R2_31 (.C(clk), 
               .CE(XLXN_35), 
               .CLR(reset), 
               .D(wdata[31]), 
               .Q(XLXN_24[31]));
   defparam R2_31.INIT = 1'b0;
   FDCE R2_32 (.C(clk), 
               .CE(XLXN_35), 
               .CLR(reset), 
               .D(wdata[32]), 
               .Q(XLXN_24[32]));
   defparam R2_32.INIT = 1'b0;
   FDCE R2_33 (.C(clk), 
               .CE(XLXN_35), 
               .CLR(reset), 
               .D(wdata[33]), 
               .Q(XLXN_24[33]));
   defparam R2_33.INIT = 1'b0;
   FDCE R2_34 (.C(clk), 
               .CE(XLXN_35), 
               .CLR(reset), 
               .D(wdata[34]), 
               .Q(XLXN_24[34]));
   defparam R2_34.INIT = 1'b0;
   FDCE R2_35 (.C(clk), 
               .CE(XLXN_35), 
               .CLR(reset), 
               .D(wdata[35]), 
               .Q(XLXN_24[35]));
   defparam R2_35.INIT = 1'b0;
   FDCE R2_36 (.C(clk), 
               .CE(XLXN_35), 
               .CLR(reset), 
               .D(wdata[36]), 
               .Q(XLXN_24[36]));
   defparam R2_36.INIT = 1'b0;
   FDCE R2_37 (.C(clk), 
               .CE(XLXN_35), 
               .CLR(reset), 
               .D(wdata[37]), 
               .Q(XLXN_24[37]));
   defparam R2_37.INIT = 1'b0;
   FDCE R2_38 (.C(clk), 
               .CE(XLXN_35), 
               .CLR(reset), 
               .D(wdata[38]), 
               .Q(XLXN_24[38]));
   defparam R2_38.INIT = 1'b0;
   FDCE R2_39 (.C(clk), 
               .CE(XLXN_35), 
               .CLR(reset), 
               .D(wdata[39]), 
               .Q(XLXN_24[39]));
   defparam R2_39.INIT = 1'b0;
   FDCE R2_40 (.C(clk), 
               .CE(XLXN_35), 
               .CLR(reset), 
               .D(wdata[40]), 
               .Q(XLXN_24[40]));
   defparam R2_40.INIT = 1'b0;
   FDCE R2_41 (.C(clk), 
               .CE(XLXN_35), 
               .CLR(reset), 
               .D(wdata[41]), 
               .Q(XLXN_24[41]));
   defparam R2_41.INIT = 1'b0;
   FDCE R2_42 (.C(clk), 
               .CE(XLXN_35), 
               .CLR(reset), 
               .D(wdata[42]), 
               .Q(XLXN_24[42]));
   defparam R2_42.INIT = 1'b0;
   FDCE R2_43 (.C(clk), 
               .CE(XLXN_35), 
               .CLR(reset), 
               .D(wdata[43]), 
               .Q(XLXN_24[43]));
   defparam R2_43.INIT = 1'b0;
   FDCE R2_44 (.C(clk), 
               .CE(XLXN_35), 
               .CLR(reset), 
               .D(wdata[44]), 
               .Q(XLXN_24[44]));
   defparam R2_44.INIT = 1'b0;
   FDCE R2_45 (.C(clk), 
               .CE(XLXN_35), 
               .CLR(reset), 
               .D(wdata[45]), 
               .Q(XLXN_24[45]));
   defparam R2_45.INIT = 1'b0;
   FDCE R2_46 (.C(clk), 
               .CE(XLXN_35), 
               .CLR(reset), 
               .D(wdata[46]), 
               .Q(XLXN_24[46]));
   defparam R2_46.INIT = 1'b0;
   FDCE R2_47 (.C(clk), 
               .CE(XLXN_35), 
               .CLR(reset), 
               .D(wdata[47]), 
               .Q(XLXN_24[47]));
   defparam R2_47.INIT = 1'b0;
   FDCE R2_48 (.C(clk), 
               .CE(XLXN_35), 
               .CLR(reset), 
               .D(wdata[48]), 
               .Q(XLXN_24[48]));
   defparam R2_48.INIT = 1'b0;
   FDCE R2_49 (.C(clk), 
               .CE(XLXN_35), 
               .CLR(reset), 
               .D(wdata[49]), 
               .Q(XLXN_24[49]));
   defparam R2_49.INIT = 1'b0;
   FDCE R2_50 (.C(clk), 
               .CE(XLXN_35), 
               .CLR(reset), 
               .D(wdata[50]), 
               .Q(XLXN_24[50]));
   defparam R2_50.INIT = 1'b0;
   FDCE R2_51 (.C(clk), 
               .CE(XLXN_35), 
               .CLR(reset), 
               .D(wdata[51]), 
               .Q(XLXN_24[51]));
   defparam R2_51.INIT = 1'b0;
   FDCE R2_52 (.C(clk), 
               .CE(XLXN_35), 
               .CLR(reset), 
               .D(wdata[52]), 
               .Q(XLXN_24[52]));
   defparam R2_52.INIT = 1'b0;
   FDCE R2_53 (.C(clk), 
               .CE(XLXN_35), 
               .CLR(reset), 
               .D(wdata[53]), 
               .Q(XLXN_24[53]));
   defparam R2_53.INIT = 1'b0;
   FDCE R2_54 (.C(clk), 
               .CE(XLXN_35), 
               .CLR(reset), 
               .D(wdata[54]), 
               .Q(XLXN_24[54]));
   defparam R2_54.INIT = 1'b0;
   FDCE R2_55 (.C(clk), 
               .CE(XLXN_35), 
               .CLR(reset), 
               .D(wdata[55]), 
               .Q(XLXN_24[55]));
   defparam R2_55.INIT = 1'b0;
   FDCE R2_56 (.C(clk), 
               .CE(XLXN_35), 
               .CLR(reset), 
               .D(wdata[56]), 
               .Q(XLXN_24[56]));
   defparam R2_56.INIT = 1'b0;
   FDCE R2_57 (.C(clk), 
               .CE(XLXN_35), 
               .CLR(reset), 
               .D(wdata[57]), 
               .Q(XLXN_24[57]));
   defparam R2_57.INIT = 1'b0;
   FDCE R2_58 (.C(clk), 
               .CE(XLXN_35), 
               .CLR(reset), 
               .D(wdata[58]), 
               .Q(XLXN_24[58]));
   defparam R2_58.INIT = 1'b0;
   FDCE R2_59 (.C(clk), 
               .CE(XLXN_35), 
               .CLR(reset), 
               .D(wdata[59]), 
               .Q(XLXN_24[59]));
   defparam R2_59.INIT = 1'b0;
   FDCE R2_60 (.C(clk), 
               .CE(XLXN_35), 
               .CLR(reset), 
               .D(wdata[60]), 
               .Q(XLXN_24[60]));
   defparam R2_60.INIT = 1'b0;
   FDCE R2_61 (.C(clk), 
               .CE(XLXN_35), 
               .CLR(reset), 
               .D(wdata[61]), 
               .Q(XLXN_24[61]));
   defparam R2_61.INIT = 1'b0;
   FDCE R2_62 (.C(clk), 
               .CE(XLXN_35), 
               .CLR(reset), 
               .D(wdata[62]), 
               .Q(XLXN_24[62]));
   defparam R2_62.INIT = 1'b0;
   FDCE R2_63 (.C(clk), 
               .CE(XLXN_35), 
               .CLR(reset), 
               .D(wdata[63]), 
               .Q(XLXN_24[63]));
   defparam R2_63.INIT = 1'b0;
   FDCE R3_0 (.C(clk), 
              .CE(XLXN_32), 
              .CLR(reset), 
              .D(wdata[0]), 
              .Q(XLXN_25[0]));
   defparam R3_0.INIT = 1'b0;
   FDCE R3_1 (.C(clk), 
              .CE(XLXN_32), 
              .CLR(reset), 
              .D(wdata[1]), 
              .Q(XLXN_25[1]));
   defparam R3_1.INIT = 1'b0;
   FDCE R3_2 (.C(clk), 
              .CE(XLXN_32), 
              .CLR(reset), 
              .D(wdata[2]), 
              .Q(XLXN_25[2]));
   defparam R3_2.INIT = 1'b0;
   FDCE R3_3 (.C(clk), 
              .CE(XLXN_32), 
              .CLR(reset), 
              .D(wdata[3]), 
              .Q(XLXN_25[3]));
   defparam R3_3.INIT = 1'b0;
   FDCE R3_4 (.C(clk), 
              .CE(XLXN_32), 
              .CLR(reset), 
              .D(wdata[4]), 
              .Q(XLXN_25[4]));
   defparam R3_4.INIT = 1'b0;
   FDCE R3_5 (.C(clk), 
              .CE(XLXN_32), 
              .CLR(reset), 
              .D(wdata[5]), 
              .Q(XLXN_25[5]));
   defparam R3_5.INIT = 1'b0;
   FDCE R3_6 (.C(clk), 
              .CE(XLXN_32), 
              .CLR(reset), 
              .D(wdata[6]), 
              .Q(XLXN_25[6]));
   defparam R3_6.INIT = 1'b0;
   FDCE R3_7 (.C(clk), 
              .CE(XLXN_32), 
              .CLR(reset), 
              .D(wdata[7]), 
              .Q(XLXN_25[7]));
   defparam R3_7.INIT = 1'b0;
   FDCE R3_8 (.C(clk), 
              .CE(XLXN_32), 
              .CLR(reset), 
              .D(wdata[8]), 
              .Q(XLXN_25[8]));
   defparam R3_8.INIT = 1'b0;
   FDCE R3_9 (.C(clk), 
              .CE(XLXN_32), 
              .CLR(reset), 
              .D(wdata[9]), 
              .Q(XLXN_25[9]));
   defparam R3_9.INIT = 1'b0;
   FDCE R3_10 (.C(clk), 
               .CE(XLXN_32), 
               .CLR(reset), 
               .D(wdata[10]), 
               .Q(XLXN_25[10]));
   defparam R3_10.INIT = 1'b0;
   FDCE R3_11 (.C(clk), 
               .CE(XLXN_32), 
               .CLR(reset), 
               .D(wdata[11]), 
               .Q(XLXN_25[11]));
   defparam R3_11.INIT = 1'b0;
   FDCE R3_12 (.C(clk), 
               .CE(XLXN_32), 
               .CLR(reset), 
               .D(wdata[12]), 
               .Q(XLXN_25[12]));
   defparam R3_12.INIT = 1'b0;
   FDCE R3_13 (.C(clk), 
               .CE(XLXN_32), 
               .CLR(reset), 
               .D(wdata[13]), 
               .Q(XLXN_25[13]));
   defparam R3_13.INIT = 1'b0;
   FDCE R3_14 (.C(clk), 
               .CE(XLXN_32), 
               .CLR(reset), 
               .D(wdata[14]), 
               .Q(XLXN_25[14]));
   defparam R3_14.INIT = 1'b0;
   FDCE R3_15 (.C(clk), 
               .CE(XLXN_32), 
               .CLR(reset), 
               .D(wdata[15]), 
               .Q(XLXN_25[15]));
   defparam R3_15.INIT = 1'b0;
   FDCE R3_16 (.C(clk), 
               .CE(XLXN_32), 
               .CLR(reset), 
               .D(wdata[16]), 
               .Q(XLXN_25[16]));
   defparam R3_16.INIT = 1'b0;
   FDCE R3_17 (.C(clk), 
               .CE(XLXN_32), 
               .CLR(reset), 
               .D(wdata[17]), 
               .Q(XLXN_25[17]));
   defparam R3_17.INIT = 1'b0;
   FDCE R3_18 (.C(clk), 
               .CE(XLXN_32), 
               .CLR(reset), 
               .D(wdata[18]), 
               .Q(XLXN_25[18]));
   defparam R3_18.INIT = 1'b0;
   FDCE R3_19 (.C(clk), 
               .CE(XLXN_32), 
               .CLR(reset), 
               .D(wdata[19]), 
               .Q(XLXN_25[19]));
   defparam R3_19.INIT = 1'b0;
   FDCE R3_20 (.C(clk), 
               .CE(XLXN_32), 
               .CLR(reset), 
               .D(wdata[20]), 
               .Q(XLXN_25[20]));
   defparam R3_20.INIT = 1'b0;
   FDCE R3_21 (.C(clk), 
               .CE(XLXN_32), 
               .CLR(reset), 
               .D(wdata[21]), 
               .Q(XLXN_25[21]));
   defparam R3_21.INIT = 1'b0;
   FDCE R3_22 (.C(clk), 
               .CE(XLXN_32), 
               .CLR(reset), 
               .D(wdata[22]), 
               .Q(XLXN_25[22]));
   defparam R3_22.INIT = 1'b0;
   FDCE R3_23 (.C(clk), 
               .CE(XLXN_32), 
               .CLR(reset), 
               .D(wdata[23]), 
               .Q(XLXN_25[23]));
   defparam R3_23.INIT = 1'b0;
   FDCE R3_24 (.C(clk), 
               .CE(XLXN_32), 
               .CLR(reset), 
               .D(wdata[24]), 
               .Q(XLXN_25[24]));
   defparam R3_24.INIT = 1'b0;
   FDCE R3_25 (.C(clk), 
               .CE(XLXN_32), 
               .CLR(reset), 
               .D(wdata[25]), 
               .Q(XLXN_25[25]));
   defparam R3_25.INIT = 1'b0;
   FDCE R3_26 (.C(clk), 
               .CE(XLXN_32), 
               .CLR(reset), 
               .D(wdata[26]), 
               .Q(XLXN_25[26]));
   defparam R3_26.INIT = 1'b0;
   FDCE R3_27 (.C(clk), 
               .CE(XLXN_32), 
               .CLR(reset), 
               .D(wdata[27]), 
               .Q(XLXN_25[27]));
   defparam R3_27.INIT = 1'b0;
   FDCE R3_28 (.C(clk), 
               .CE(XLXN_32), 
               .CLR(reset), 
               .D(wdata[28]), 
               .Q(XLXN_25[28]));
   defparam R3_28.INIT = 1'b0;
   FDCE R3_29 (.C(clk), 
               .CE(XLXN_32), 
               .CLR(reset), 
               .D(wdata[29]), 
               .Q(XLXN_25[29]));
   defparam R3_29.INIT = 1'b0;
   FDCE R3_30 (.C(clk), 
               .CE(XLXN_32), 
               .CLR(reset), 
               .D(wdata[30]), 
               .Q(XLXN_25[30]));
   defparam R3_30.INIT = 1'b0;
   FDCE R3_31 (.C(clk), 
               .CE(XLXN_32), 
               .CLR(reset), 
               .D(wdata[31]), 
               .Q(XLXN_25[31]));
   defparam R3_31.INIT = 1'b0;
   FDCE R3_32 (.C(clk), 
               .CE(XLXN_32), 
               .CLR(reset), 
               .D(wdata[32]), 
               .Q(XLXN_25[32]));
   defparam R3_32.INIT = 1'b0;
   FDCE R3_33 (.C(clk), 
               .CE(XLXN_32), 
               .CLR(reset), 
               .D(wdata[33]), 
               .Q(XLXN_25[33]));
   defparam R3_33.INIT = 1'b0;
   FDCE R3_34 (.C(clk), 
               .CE(XLXN_32), 
               .CLR(reset), 
               .D(wdata[34]), 
               .Q(XLXN_25[34]));
   defparam R3_34.INIT = 1'b0;
   FDCE R3_35 (.C(clk), 
               .CE(XLXN_32), 
               .CLR(reset), 
               .D(wdata[35]), 
               .Q(XLXN_25[35]));
   defparam R3_35.INIT = 1'b0;
   FDCE R3_36 (.C(clk), 
               .CE(XLXN_32), 
               .CLR(reset), 
               .D(wdata[36]), 
               .Q(XLXN_25[36]));
   defparam R3_36.INIT = 1'b0;
   FDCE R3_37 (.C(clk), 
               .CE(XLXN_32), 
               .CLR(reset), 
               .D(wdata[37]), 
               .Q(XLXN_25[37]));
   defparam R3_37.INIT = 1'b0;
   FDCE R3_38 (.C(clk), 
               .CE(XLXN_32), 
               .CLR(reset), 
               .D(wdata[38]), 
               .Q(XLXN_25[38]));
   defparam R3_38.INIT = 1'b0;
   FDCE R3_39 (.C(clk), 
               .CE(XLXN_32), 
               .CLR(reset), 
               .D(wdata[39]), 
               .Q(XLXN_25[39]));
   defparam R3_39.INIT = 1'b0;
   FDCE R3_40 (.C(clk), 
               .CE(XLXN_32), 
               .CLR(reset), 
               .D(wdata[40]), 
               .Q(XLXN_25[40]));
   defparam R3_40.INIT = 1'b0;
   FDCE R3_41 (.C(clk), 
               .CE(XLXN_32), 
               .CLR(reset), 
               .D(wdata[41]), 
               .Q(XLXN_25[41]));
   defparam R3_41.INIT = 1'b0;
   FDCE R3_42 (.C(clk), 
               .CE(XLXN_32), 
               .CLR(reset), 
               .D(wdata[42]), 
               .Q(XLXN_25[42]));
   defparam R3_42.INIT = 1'b0;
   FDCE R3_43 (.C(clk), 
               .CE(XLXN_32), 
               .CLR(reset), 
               .D(wdata[43]), 
               .Q(XLXN_25[43]));
   defparam R3_43.INIT = 1'b0;
   FDCE R3_44 (.C(clk), 
               .CE(XLXN_32), 
               .CLR(reset), 
               .D(wdata[44]), 
               .Q(XLXN_25[44]));
   defparam R3_44.INIT = 1'b0;
   FDCE R3_45 (.C(clk), 
               .CE(XLXN_32), 
               .CLR(reset), 
               .D(wdata[45]), 
               .Q(XLXN_25[45]));
   defparam R3_45.INIT = 1'b0;
   FDCE R3_46 (.C(clk), 
               .CE(XLXN_32), 
               .CLR(reset), 
               .D(wdata[46]), 
               .Q(XLXN_25[46]));
   defparam R3_46.INIT = 1'b0;
   FDCE R3_47 (.C(clk), 
               .CE(XLXN_32), 
               .CLR(reset), 
               .D(wdata[47]), 
               .Q(XLXN_25[47]));
   defparam R3_47.INIT = 1'b0;
   FDCE R3_48 (.C(clk), 
               .CE(XLXN_32), 
               .CLR(reset), 
               .D(wdata[48]), 
               .Q(XLXN_25[48]));
   defparam R3_48.INIT = 1'b0;
   FDCE R3_49 (.C(clk), 
               .CE(XLXN_32), 
               .CLR(reset), 
               .D(wdata[49]), 
               .Q(XLXN_25[49]));
   defparam R3_49.INIT = 1'b0;
   FDCE R3_50 (.C(clk), 
               .CE(XLXN_32), 
               .CLR(reset), 
               .D(wdata[50]), 
               .Q(XLXN_25[50]));
   defparam R3_50.INIT = 1'b0;
   FDCE R3_51 (.C(clk), 
               .CE(XLXN_32), 
               .CLR(reset), 
               .D(wdata[51]), 
               .Q(XLXN_25[51]));
   defparam R3_51.INIT = 1'b0;
   FDCE R3_52 (.C(clk), 
               .CE(XLXN_32), 
               .CLR(reset), 
               .D(wdata[52]), 
               .Q(XLXN_25[52]));
   defparam R3_52.INIT = 1'b0;
   FDCE R3_53 (.C(clk), 
               .CE(XLXN_32), 
               .CLR(reset), 
               .D(wdata[53]), 
               .Q(XLXN_25[53]));
   defparam R3_53.INIT = 1'b0;
   FDCE R3_54 (.C(clk), 
               .CE(XLXN_32), 
               .CLR(reset), 
               .D(wdata[54]), 
               .Q(XLXN_25[54]));
   defparam R3_54.INIT = 1'b0;
   FDCE R3_55 (.C(clk), 
               .CE(XLXN_32), 
               .CLR(reset), 
               .D(wdata[55]), 
               .Q(XLXN_25[55]));
   defparam R3_55.INIT = 1'b0;
   FDCE R3_56 (.C(clk), 
               .CE(XLXN_32), 
               .CLR(reset), 
               .D(wdata[56]), 
               .Q(XLXN_25[56]));
   defparam R3_56.INIT = 1'b0;
   FDCE R3_57 (.C(clk), 
               .CE(XLXN_32), 
               .CLR(reset), 
               .D(wdata[57]), 
               .Q(XLXN_25[57]));
   defparam R3_57.INIT = 1'b0;
   FDCE R3_58 (.C(clk), 
               .CE(XLXN_32), 
               .CLR(reset), 
               .D(wdata[58]), 
               .Q(XLXN_25[58]));
   defparam R3_58.INIT = 1'b0;
   FDCE R3_59 (.C(clk), 
               .CE(XLXN_32), 
               .CLR(reset), 
               .D(wdata[59]), 
               .Q(XLXN_25[59]));
   defparam R3_59.INIT = 1'b0;
   FDCE R3_60 (.C(clk), 
               .CE(XLXN_32), 
               .CLR(reset), 
               .D(wdata[60]), 
               .Q(XLXN_25[60]));
   defparam R3_60.INIT = 1'b0;
   FDCE R3_61 (.C(clk), 
               .CE(XLXN_32), 
               .CLR(reset), 
               .D(wdata[61]), 
               .Q(XLXN_25[61]));
   defparam R3_61.INIT = 1'b0;
   FDCE R3_62 (.C(clk), 
               .CE(XLXN_32), 
               .CLR(reset), 
               .D(wdata[62]), 
               .Q(XLXN_25[62]));
   defparam R3_62.INIT = 1'b0;
   FDCE R3_63 (.C(clk), 
               .CE(XLXN_32), 
               .CLR(reset), 
               .D(wdata[63]), 
               .Q(XLXN_25[63]));
   defparam R3_63.INIT = 1'b0;
   D2_4E_MXILINX_register_file XLXI_10 (.A0(waddr0), 
                                        .A1(waddr1), 
                                        .E(XLXN_13), 
                                        .D0(XLXN_5), 
                                        .D1(XLXN_9), 
                                        .D2(XLXN_7), 
                                        .D3(XLXN_8));
   // synthesis attribute HU_SET of XLXI_10 is "XLXI_10_0"
   AND2 XLXI_11 (.I0(XLXN_5), 
                 .I1(wen), 
                 .O(XLXN_33));
   AND2 XLXI_12 (.I0(XLXN_9), 
                 .I1(wen), 
                 .O(XLXN_34));
   AND2 XLXI_13 (.I0(XLXN_7), 
                 .I1(wen), 
                 .O(XLXN_35));
   AND2 XLXI_14 (.I0(XLXN_8), 
                 .I1(wen), 
                 .O(XLXN_32));
   VCC XLXI_15 (.P(XLXN_13));
   raddrmux4_1 XLXI_20 (.addr(r0addr[1:0]), 
                        .R0(XLXN_19[63:0]), 
                        .R1(XLXN_22[63:0]), 
                        .R2(XLXN_24[63:0]), 
                        .R3(XLXN_25[63:0]), 
                        .o(r0data[63:0]));
   raddrmux4_1 XLXI_21 (.addr(r1addr[1:0]), 
                        .R0(XLXN_19[63:0]), 
                        .R1(XLXN_22[63:0]), 
                        .R2(XLXN_24[63:0]), 
                        .R3(XLXN_25[63:0]), 
                        .o(r1data[63:0]));
endmodule
