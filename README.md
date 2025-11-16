# RISC-V Processor Simulator

This repository contains a basic single-cycle RISC-V processor implemented in Verilog, along with a testbench for simulation. The processor supports a subset of RISC-V instructions, including arithmetic (add, sub, addi), logical (and, or, xor), shifts (sll, srl, sra), memory (lw, sw), control (beq, bne, jal, jalr), and immediate/utility (lui, auipc).

## Features
- Single-cycle datapath.
- Supports RV32I base instruction set (partial implementation as specified).
- Testbench with sample program demonstrating basic operations.
- Simulation using open-source tools from OSS CAD Suite (Icarus Verilog + GTKWave).

## Prerequisites
- A Unix-like environment (Linux, macOS) or Windows.
- Basic knowledge of Verilog and RISC-V architecture.

## Setup and Installation

### Step 1: Clone the Repository
```bash
git clone <your-repo-url>
cd <repo-name>
```

### Step 2: Install OSS CAD Suite (OPTIONAL, BUT RECOMMENDED)
This project uses the OSS CAD Suite for simulation and waveform viewing, which includes Icarus Verilog (for compilation and simulation) and GTKWave (for viewing waveforms). The suite is open-source and provided by YosysHQ.

1. Visit the GitHub releases page: [https://github.com/YosysHQ/oss-cad-suite-build/releases/latest](https://github.com/YosysHQ/oss-cad-suite-build/releases/latest).
2. Download the pre-built archive for your operating system:
   - Linux (x64 or ARM64).
   - macOS (Intel x64 or ARM64 for M1/M2/M3).
   - Windows (x64).
3. Extract the archive to a directory (e.g., `~/oss-cad-suite`).

4. Set up your environment to access the tools:
   - **Linux/macOS (Bash/Zsh)**:
     ```bash
     export PATH="$HOME/oss-cad-suite/bin:$PATH"
     ```
     Add this to your `~/.bashrc` or `~/.zshrc` for persistence.
   - **macOS (Fish shell)**:
     ```fish
     fish_add_path "$HOME/oss-cad-suite/bin"
     ```
   - **Windows (CMD)**:
     Run:
     ```
     <extracted_path>\oss-cad-suite\environment.bat
     ```
   - **Windows (PowerShell)**:
     Run:
     ```
     . <extracted_path>\oss-cad-suite\environment.ps1
     ```
   - Alternatively, on Windows, double-click `<extracted_path>\oss-cad-suite\start.bat` to open a pre-configured command prompt.

5. Verify installation:
   ```bash
   iverilog -v
   gtkwave --version
   ```
   These should display version information.

**Note**: On macOS, if you encounter a quarantine error, run `xattr -d com.apple.quarantine <downloaded_file>.tgz` before extracting.

### Step 3: Prepare the Verilog Files
- The main design is in `RISC-V.v`.

## Usage Instructions

### Compiling, Simulating, and Waveform Viewing

1. **Add VCD Dumping to Testbench (if not already present)**:
   Open the testbench section in your Verilog file and add the following inside the `initial begin` block (after loading instructions):
   ```verilog
   $dumpfile("riscv_dump.vcd");
   $dumpvars(0, tb_top);
   ```

2. **Compile the Design**:
   ```bash
   iverilog -o riscv.vvp riscv.v
   ```
   - This generates a simulation executable `riscv.vvp`.
   - Use `-Wall` for warnings: `iverilog -Wall -o riscv.vvp riscv.v`.

3. **Run the Simulation**:
   ```bash
   vvp riscv.vvp
   ```
   - This executes the testbench, prints console output (e.g., register values via `$display` and `$monitor`), and generates `riscv_dump.vcd` if dumping is enabled.

4. **View Waveforms in GTKWave**:
   ```bash
   gtkwave riscv_dump.vcd
   ```
   - In GTKWave:
     - Browse the signal hierarchy on the left.
     - Drag signals (e.g., `clk`, `PC_top`, `instruction_top`, or registers like `uut.Reg_File.Registers[1]`) to the "Signals" panel.
     - Use zoom tools and markers to inspect the waveform.
   - Save your signal configuration as a `.gtkw` file for reuse (e.g., `gtkwave riscv_dump.vcd -S signals.gtkw`).

**Simulation Notes**:
- The default testbench runs for 120 time units (#120). Adjust this delay if your program needs more cycles.
- Each clock cycle is 10 time units (#5 for high, #5 for low in the `always` block).

## Customizing the Testbench

### Changing the Test Program
The testbench loads instructions into instruction memory (`IMemory`) as machine code (hex values). To run a different program:

1. Write your RISC-V assembly code (e.g., in a file `test.asm`).
2. Assemble it into machine code:
   - Use an online RISC-V assembler like [Venus Simulator](https://venus.cs61c.org/) or install `riscv-gnu-toolchain` locally.
   - Example command (if toolchain installed): `riscv64-unknown-elf-as test.asm -o test.o; riscv64-unknown-elf-objdump -d test.o`.
   - Extract the hex opcodes from the output.

3. Update the testbench's `initial begin` block:
   ```verilog
   uut.Inst_Memory.IMemory[0] = 32'h<opcode1>; // Instruction 1
   uut.Inst_Memory.IMemory[1] = 32'h<opcode2>; // Instruction 2
   // ...
   ```
   - Addresses are word-aligned (divided by 4 for array index).
   - Adjust the simulation time (`#120;`) based on your program's length.

4. Recompile and simulate as above.

### Converting Assembly to Machine Code Using AI
If you don't have an assembler handy:
- Use an AI tool like Grok, ChatGPT, or Claude.
- Prompt example: "Convert this RISC-V assembly to 32-bit hex machine code: addi x1, x0, 5; addi x2, x0, 10;"
- It will output hex values (e.g., 32'h00500093).
- Verify the output with a simulator if possible.
- Paste the hex into the testbench as shown above.

### Adding More Instructions or Modifying the Design
- To support additional instructions, update:
  - `ImmGen`: Add immediate formats.
  - `Control_Unit`: Set control signals for new opcodes.
  - `ALU_Control` and `ALU_unit`: Add ALU operations.
  - `Branch_logic`: For more branch types.
- Test with a new program in the testbench.

## Troubleshooting
- **Compilation Errors**: Check syntax (e.g., missing semicolons, wire declarations). Use `iverilog -g2012` if SystemVerilog features are used.
- **No Output**: Ensure reset is deasserted and clock is toggling.
- **Large VCD Files**: Use `$dumpvars(1, tb_top);` for shallower dumping.
- **Alternative Simulator**: For faster simulation, try Verilator (also in OSS CAD Suite). Example: `verilator --trace -cc riscv.v --exe tb.cpp` (requires a C++ wrapper `tb.cpp`).

## License
This project is licensed under the MIT License.

## AI Usage
- AI was used for debugging and creating the testbench from the given sample test assembly program:
  ```asm
  .section .text
  .globl _start
  _start:
  addi x1, x0, 5 # x1 = 5
  addi x2, x0, 10 # x2 = 10
  add x3, x1, x2 # x3 = 15
  sub x4, x2, x1 # x4 = 5
  lui x5, 0x00010 # x5 = 0x0001_0000 (data base)
  sw x3, 0(x5) # mem[0x0001_0000] = 15
  lw x4, 0(x5) # x4 = 15
  beq x3, x4, label1 # branch forward by 8 bytes
  addi x6, x0, 1 # skipped if branch taken
  label1:
  addi x6, x0, 2 # x6 = 2
  jal x0, 0 # halt: infinite loop
  ```
## Acknowledgments
- Based on standard RISC-V single-cycle design.
- Major inspiration from this [video](https://youtu.be/dh88oe6O0QU?si=sq2HffXmpy5Kaef4).
- Tools from [YosysHQ OSS CAD Suite](https://github.com/YosysHQ/oss-cad-suite-build).

