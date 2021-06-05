library verilog;
use verilog.vl_types.all;
entity extoma is
    port(
        clk             : in     vl_logic;
        reset           : in     vl_logic;
        regwrite_ex     : in     vl_logic;
        memtoreg_ex     : in     vl_logic;
        memwrite_ex     : in     vl_logic;
        aluout_ex       : in     vl_logic_vector(31 downto 0);
        writedata_ex_fwd: in     vl_logic_vector(31 downto 0);
        writereg_ex     : in     vl_logic_vector(4 downto 0);
        pcplus4_ex      : in     vl_logic_vector(31 downto 0);
        jump_ex         : in     vl_logic;
        pc_ex           : in     vl_logic_vector(31 downto 0);
        instr_ex        : in     vl_logic_vector(31 downto 0);
        regwrite_ma     : out    vl_logic;
        memtoreg_ma     : out    vl_logic;
        memwrite_ma     : out    vl_logic;
        aluout_ma       : out    vl_logic_vector(31 downto 0);
        writedata_ma    : out    vl_logic_vector(31 downto 0);
        writereg_ma     : out    vl_logic_vector(4 downto 0);
        pcplus4_ma      : out    vl_logic_vector(31 downto 0);
        jump_ma         : out    vl_logic;
        pc_ma           : out    vl_logic_vector(31 downto 0);
        instr_ma        : out    vl_logic_vector(31 downto 0)
    );
end extoma;
