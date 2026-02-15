import sys
import os
import subprocess
import time
from base_opterm import openterm

n = 5
eth_n0 = "10.0.4.2"
eth_n1 = "10.0.5.2"
eth_n2 = "10.0.6.2"
eth_n3 = "10.0.7.2"
# node_n0 = "$n0"
# node_n1 = "$n1"
# node_n2 = "$n2"
# node_n3 = "$n3"
node_n0 = "10.0.4.3"
node_n1 = "10.0.5.3"
node_n2 = "10.0.6.3"
node_n3 = "10.0.7.3"
port = 5020


count = 0
offset = 21
RF_WIDTH = 3
DMEM_WIDTH = 3
cmd_dist={ 
    "d_w" : "dmem_write",
    "d_r" : "dmem_read",
    "reset" : "pcreset",
    'run' : 'run 1',
    'step' : 'step',




    'lw' : 'imem_write',
    'sw' : 'imem_write',
    'nop' : 'imem_write'

}
upcode_dist = {
    'nop' : '00',
    'lw' : '01',
    'sw' : '10'



}
# d_w $addr data
# d_r $addr
# reset
# lw $reg $addr
# sw $reg $addr



BF_MAP = {
    'nic': '/home/netfpga/bitfiles/reference_nic.bit',
    'router': '/home/netfpga/bitfiles/reference_router.bit',
    'ids':  "bitfiles/nf2_top_par_ids.bit",
    'pipeline': '/home/netfpga/ykl/nf2_top_par.bit'
}
PERL_SCRIPT_MAP = {
    'ids': './idsreg',
    'pipeline': './ykl/pipereg.pl'
}




def de_hex(str):
    if str.startswith('0x'):
        return hex(int(str, 16))
    elif str.startswith('$'):
        str = str[1:]
        return hex(int(str))
    else:
        return hex(int(str))

def hex_bi(str, width=32):
    value = de_hex(str)
    value = int(value, 16)
    return f'{value:0{width}b}'

def split(str):
    value=de_hex(str)
    high = (value >> 32) & 0xFFFFFFFF
    low = value & 0xFFFFFFFF
    return high, low

def build_instr(upcode, reg1, reg2, wreg, offset, line):
    bi_instr = f'{upcode}' + f'{reg1}' + f'{reg2}'+ f'{wreg}' + '0' * offset
    if len(bi_instr) != 32:
        raise ValueError(f"Line {line}:  {bi_instr} Instruction length is not 32 bits")
    hex_instr = hex(int(bi_instr, 2))
    return hex_instr

       

def logging(file, result):
    
    with open(file, 'a') as f:
        f.write(result + '\n')

def open_file(address):
    try:
        with open(address, 'r') as f:           
            contents = [[line.strip()] for line in f ] 
    except FileNotFoundError:
        raise (f"Error: File '{address}' not found.")
        
    except Exception as e:
        raise (f"Error reading file '{address}': {e}")

    return contents

def decode (command, script, log=False):
    instrs = []
    lines = []
    if log:
            file = f'instr_perl_based.log'
            if os.path.exists(file):
                os.remove(file)
                print(f"{file} removed.")
    #          skip #
    #         comment_index = instr.find('#')
    #         if comment_index != -1:
    #             instr = instr[:comment_index].strip()  # Remove comments and whitespace
    #         instrs.append(instr)
    for line in range(len(command)):
        instr = command[line][0]
        instr_decode = ''
        comment_index = instr.find('#')
        if comment_index != -1:
            instr = instr[:comment_index]  # Remove comments and whitespace
        instr = instr.replace(';','')
        instr = instr.strip()
        if not instr:
            continue  # Skip empty lines
        if instr[0].isdigit():
            instr_decode = f'{script} imem_write {instr}'
        else:
            split_in = instr.split()
            # if split_in[0] == 'dmem_w':
            #     if len(split_in) < 3:
            #         raise ValueError(f"Invalid instruction in line {line}: {instr}")
            #     addr = de_hex(split_in[1])
            #     high, low = split(split_in[2])
            #     instr_decode = f'dmem_write {addr} {high} {low}'     
            # else:
            instr_decode =mapping(split_in, line, script)
        # instr_decode = f'{script} {instr_decode}'
        instrs.append(instr_decode)
        lines.append(line)
        if log:
            logging(file, f'{instr_decode}\n')
    return instrs, lines


def pipeline_logic(instrs, lines, bitfile, script):
    subprocess.run(['tmux', 'send-keys', '-t', 'nd0', f'nf_download {BF_MAP.get(bitfile,bitfile)}', 'C-m'])
    time.sleep(10)
    subprocess.run(['tmux', 'send-keys', '-t', 'nd0', 'rkd &', 'C-m'])
    time.sleep(1)
    reset = mapping(['reset'], 'reset', script)
    subprocess.run(['tmux', 'send-keys', '-t', 'nd0', reset, 'C-m'])
    time.sleep(1)
    for i in range(len(instrs)):
        subprocess.run(['tmux', 'send-keys', '-t', 'nd0', instrs[i], 'C-m'])
        time.sleep(0.1)
    print ("All instructions sent. Waiting for execution ...")
    while 1:
        input_str = input("Enter 'run' 'step' 'q' or other command: ").strip()
        if input_str == 'run':
            subprocess.run(['tmux', 'send-keys', '-t', 'nd0', f'{mapping(["run"], "execution start", script)}', 'C-m'])
        elif input_str == 'step':
            subprocess.run(['tmux', 'send-keys', '-t', 'nd0', f'{mapping(["step"], "step execution", script)}', 'C-m'])
        elif input_str == 'q':
            print("Exiting...")
            subprocess.run(['tmux', 'send-keys', '-t', 'nd0', 'killall rkd', 'C-m'])
            break
        else:
            subprocess.run(['tmux', 'send-keys', '-t', 'nd0', f'{script} {input_str}', 'C-m'])
            print(" unknown command, sent to terminal script")


    



def mapping(split_in, line, script):
    global count
    try:
        cmd = cmd_dist.get(split_in[0])
    except KeyError:
        raise ValueError(f"Unknown command in line {line}: {split_in[0]}")
    except Exception as e:
        raise ValueError(f"Error decode line {line}: {e}")
    bi_instr = ''
    if cmd == 'dmem_write':
        if len(split_in) < 3:
            raise ValueError(f"Invalid instruction in line {line}: {' '.join(split_in)}")
        addr = de_hex(split_in[1])
        high, low = split(addr)
        decode_in =  f'{cmd} {addr} {high} {low}'
    elif cmd == 'dmem_read':
        if len(split_in) < 2:
            raise ValueError(f"Invalid instruction in line {line}: {' '.join(split_in)}")
        addr = de_hex(split_in[1])
        decode_in = f'{cmd} {addr}'
    elif cmd == 'pcreset':
        decode_in = f'{cmd}'
    elif cmd == 'run 1':
        decode_in = f'{cmd}'
    elif cmd == 'step':
        decode_in = f'{cmd}'
    elif cmd == 'imem_write':
        upcode = upcode_dist.get(split_in[0])
        if split_in[0] == 'nop':
            bi_instr = build_instr(upcode, '000', '000', '000', offset, line)
        elif split_in[0] == 'lw' :
            if len(split_in) < 3:
                raise ValueError(f"Invalid instruction in line {line}: {' '.join(split_in)}")
            reg = hex_bi(split_in[1],width=RF_WIDTH)
            addr = hex_bi(split_in[2],width=DMEM_WIDTH)
            bi_instr = build_instr(upcode, addr, '000', reg, offset, line)
        elif split_in[0] == 'sw' :
            if len(split_in) < 3:
                raise ValueError(f"Invalid instruction in line {line}: {' '.join(split_in)}")
            reg = hex_bi(split_in[1],width=RF_WIDTH)
            addr = hex_bi(split_in[2],width=DMEM_WIDTH)
            bi_instr = build_instr(upcode,addr, reg, '000', offset, line)


        decode_in = f'{cmd} {count} {bi_instr}'
        count += 1
    else:
        raise ValueError(f"Unknown command in line {line}: {split_in[0]}")
    decode_in = f'{script} {decode_in}'

    return decode_in
    



    






def main():
    global count
    if len(sys.argv) < 6:
        raise ValueError("Usage: python mission_send.py <name> <password> <bitfile> <cmdfile> <scripts>")
    openterm(sys.argv, n)
    bitfile = BF_MAP.get(sys.argv[3], sys.argv[3])
    command_file = sys.argv[4]
    script = PERL_SCRIPT_MAP.get(sys.argv[5], sys.argv[5])
    command = open_file(command_file)
    instrs, lines = decode(command, script, log = True)
    pipeline_logic(instrs, lines, bitfile, script)



    
    
