library verilog;
use verilog.vl_types.all;
entity matowb is
    port(
        clk             : in     vl_logic;
        reset           : in     vl_logic;
        regwrite_ma     : in     vl_logic;
        memtoreg_ma     : in     vl_logic;
        readdata_ma     : in     vl_logic_vector(31 downto 0);
        aluout_ma       : in     vl_logic_vector(31 downto 0);
        pcplus4_ma      : in     vl_logic_vector(31 downto 0);
        jump_ma         : in     vl_logic;
        pc_ma           : in     vl_logic_vector(31 downto 0);
        instr_ma        : in     vl_logic_vector(31 downto 0);
        writereg_ma     : in     vl_logic_vector(4 downto 0);
        regwrite_wb     : out    vl_logic;
        memtoreg_wb     : out    vl_logic;
        readdata_wb     : out    vl_logic_vector(31 downto 0);
        aluout_wb       : out    vl_logic_vector(31 downto 0);
        pcplus4_wb      : out    vl_logic_vector(31 downto 0);
        jump_wb         : out    vl_logic;
        pc_wb           : out    vl_logic_vector(31 downto 0);
        instr_wb        : out    vl_logic_vector(31 downto 0);
        writereg_wb     : out    vl_logic_vector(4 downto 0)
    );
end matowb;
