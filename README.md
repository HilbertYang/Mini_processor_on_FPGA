# Mini Processor on FPGA

**multi-stage pipelined processor** designed for netFPGA implementation, featuring custom memory mapping and optimized storage interfaces.

## Key Features

* **Multi-Stage Pipeline**: Optimized instruction execution flow to enhance throughput and clock frequency.
* **Memory Architecture**:
    * **I-Mem & D-Mem**: Integrated via FPGA **IP Cores**, supporting efficient **Synchronous Read/Write** operations.
    * **Register File**: Custom design featuring **Asynchronous Read** and **Synchronous Write** to maintain pipeline timing integrity.
* **Software-Hardware Interface**: Implemented via **Register Mapping**, allowing software instructions to interact directly with hardware interfaces.
* **Custom ISA**: Supports basic computational, logic, and control-flow operations.

## Architecture Overview

The processor follows a classic pipelined stage design (Fetch, Decode, Execute, Memory, Write-back) with specific hardware optimizations:

1.  **Memory Access**: Synchronous interfaces with Data Memory and Instruction Memory IP cores to ensure stability at higher clock speeds.
2.  **Register Mapping**: Dedicated address space for I/O and control registers, enabling easy software-driven hardware control.
3.  **Hazard Handling**: (Optional: Mention if you have implemented Forwarding or Stalling logic here).

## Project Structure

* `/src`: Verilog source code for the pipeline logic.
* `/bin`: Bitfiles for FPGA.
* `/lib`: SW regs maping to the HW regs .
* `/sim`: Simulation files to verify the processor logic.
