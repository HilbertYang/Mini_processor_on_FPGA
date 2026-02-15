use strict;
my $PIPE_BASE = 0x2000240;
#SW regs
my $PIPE_CTRL_REG          = $PIPE_BASE + 0x0;
my $PIPE_IMEM_ADDR_REG     = $PIPE_BASE + 0x4;
my $PIPE_IMEM_WDATA_REG    = $PIPE_BASE + 0x8;
my $PIPE_DMEM_ADDR_REG     = $PIPE_BASE + 0xc;
my $PIPE_DMEM_WDATA_LO_REG = $PIPE_BASE + 0x10;
my $PIPE_DMEM_WDATA_HI_REG = $PIPE_BASE + 0x14;
my $PIPE_RESERVED_REG       = $PIPE_BASE + 0x18;
#HW dbg regs
my $PIPE_PC_DBG_REG        = $PIPE_BASE + 0x1c;
my $PIPE_IF_INSTR_REG      = $PIPE_BASE + 0x20;
my $PIPE_DMEM_RDATA_LO_REG = $PIPE_BASE + 0x24;
my $PIPE_DMEM_RDATA_HI_REG = $PIPE_BASE + 0x28;

##########################################################################
####################### READ/WRITE HELPERS ###############################
###########################################################################
sub regwrite {
  my($addr, $value) = @_;
  my $cmd = sprintf("regwrite 0x%08x 0x%08x", $addr, $value);
  `$cmd`;
}

sub regread {
  my($addr) = @_;
  my $cmd = sprintf("regread 0x%08x", $addr);
  my @out = `$cmd`;
  my $result = $out[0];
  if ($result =~ m/Reg (0x[0-9a-f]+) \((\d+)\):\s+(0x[0-9a-f]+) \((\d+)\)/i) {
    return $3;
  }
  return $result;
}
# OUTPUT: Reg 0xADDR (DEC): 0xVALUE (DEC)

###########################################################################
############################ PIPE REG CMDS ################################
###########################################################################
sub usage {
  print "Usage: pipereg <cmd> [args]\n";
  print "  Commands:\n";
  print "    run <0|1>                                   set run\n";
  print "    step                                        single step\n";
  print "    pcreset                                     pc_reset_pulse\n";
  print "    imem_write <addr> <wdata>                   program I-mem word\n";
  print "    dmem_write <addr> <hi> <lo>                 program D-mem 64b\n";
  print "    dmem_read <addr>                            read D-mem 64b via portB\n";
  print "    dbg                                         print pc + if_instr\n";
  print "    allregs                                     dump all hw regs\n";
}

  # CTRL bits
  # wire run_level      =  sw_ctrl[0];
  # wire step_pulse     =  sw_ctrl[1] & ~sw_ctrl_d[1];
  # wire pc_reset_pulse =  sw_ctrl[2] & ~sw_ctrl_d[2];
  # wire imem_we_pulse  =  sw_ctrl[3] & ~sw_ctrl_d[3];
  # wire dmem_prog_en   =  sw_ctrl[4];
  # wire dmem_prog_we   =  sw_ctrl[5];

sub ctrl_read_val {
  my $v = regread($PIPE_CTRL_REG);
  $v =~ s/\s+//g;
  return hex($v);
}

sub ctrl_write_val {
  my($v) = @_;
  regwrite($PIPE_CTRL_REG, $v);
}

sub ctrl_set_bit {
  my($bit, $val) = @_;
  my $v = ctrl_read_val();
  if ($val) {
    $v |= (1 << $bit); 
    }
    else{
    $v &= ~(1 << $bit); 
    }
  ctrl_write_val($v);
}

sub ctrl_pulse_bit {
  my($bit) = @_;
  ctrl_set_bit($bit, 0);
  ctrl_set_bit($bit, 1);
  ctrl_set_bit($bit, 0);
}

sub cmd_run {
  my($on) = @_;
  ctrl_set_bit(0, $on ? 1 : 0); # run_level
}

sub cmd_step {
  ctrl_pulse_bit(1); # step_pulse
}

sub cmd_pcreset {
  ctrl_pulse_bit(2); # pc_reset_pulse
}

sub cmd_imem_write {
  my($addr, $wdata) = @_;
  my $a = ($addr =~ /^0x/i) ? hex($addr) : int($addr);
  my $d = ($wdata =~ /^0x/i) ? hex($wdata) : hex("0x$wdata");
  regwrite($PIPE_IMEM_ADDR_REG, $a);
  regwrite($PIPE_IMEM_WDATA_REG, $d);
  ctrl_pulse_bit(3); # imem_we 
}

sub cmd_dmem_write {
  my($addr, $hi, $lo) = @_;
  my $a  = ($addr =~ /^0x/i) ? hex($addr) : int($addr);
  my $hi_v = ($hi =~ /^0x/i) ? hex($hi) : hex("0x$hi");
  my $lo_v = ($lo =~ /^0x/i) ? hex($lo) : hex("0x$lo");
  regwrite($PIPE_DMEM_ADDR_REG, $a);
  regwrite($PIPE_DMEM_WDATA_HI_REG, $hi_v);
  regwrite($PIPE_DMEM_WDATA_LO_REG, $lo_v);
  ctrl_set_bit(4, 1); # dmem_en=1
  ctrl_set_bit(5, 1); # dmem_we=1
  ctrl_set_bit(5, 0); # dmem_we=0
}

sub cmd_dmem_read {
  my($addr) = @_;
  my $a  = ($addr =~ /^0x/i) ? hex($addr) : int($addr);
  regwrite($PIPE_DMEM_ADDR_REG, $a);
  ctrl_set_bit(4, 1); # dmem_en=1
  ctrl_set_bit(5, 0); # dmem_we=0
  my $lo = regread($PIPE_DMEM_RDATA_LO_REG);
  my $hi = regread($PIPE_DMEM_RDATA_HI_REG);
  print "DMEM[$a] = $hi$lo\n";
}

sub cmd_dbg {
  print "PC:       ", regread($PIPE_PC_DBG_REG), "\n";
  print "IF_INSTR: ", regread($PIPE_IF_INSTR_REG), "\n";
}

sub cmd_allregs {
  cmd_dbg();
  print "DMEM_RLO: ", regread($PIPE_DMEM_RDATA_LO_REG), "\n";
  print "DMEM_RHI: ", regread($PIPE_DMEM_RDATA_HI_REG), "\n";
}

#=======================MAIN===========================
my $numargs = $#ARGV + 1;
if ($numargs < 1) {
  usage();
  exit(1);
  }

my $cmd = $ARGV[0];

if ($cmd eq "run") {
  die "run <0|1>\n" if $numargs < 2;
  cmd_run($ARGV[1]);
}
elsif ($cmd eq "step") {
  cmd_step();
}
elsif ($cmd eq "pcreset") {
  cmd_pcreset();
}
elsif ($cmd eq "imem_write") {
  die "imem_write <addr> <wdata>\n" if $numargs < 3;
  cmd_imem_write($ARGV[1], $ARGV[2]);
}
elsif ($cmd eq "dmem_write") {
  die "dmem_write <addr> <hi> <lo>\n" if $numargs < 4;
  cmd_dmem_write($ARGV[1], $ARGV[2], $ARGV[3]);
}
elsif ($cmd eq "dmem_read") {
  die "dmem_read <addr>\n" if $numargs < 2;
  cmd_dmem_read($ARGV[1]);
}
elsif ($cmd eq "dbg") {
  cmd_dbg();
}
elsif ($cmd eq "allregs") {
  cmd_allregs();
}
else {
  print "Unrecognized command $cmd\n";
  usage();
  exit(1);
}
