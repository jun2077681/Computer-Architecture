library verilog;
use verilog.vl_types.all;
entity iftoid is
    port(
        clk             : in     vl_logic;
        reset           : in     vl_logic;
        stall           : in     vl_logic;
        pcsrc           : in     vl_logic;
        jump_ex         : in     vl_logic;
        jrcontrol_ex    : in     vl_logic;
        pcplus4_if      : in     vl_logic_vector(31 downto 0);
        instr_if        : in     vl_logic_vector(31 downto 0);
        pc_if           : in     vl_logic_vector(31 downto 0);
        pcplus4_id      : out    vl_logic_vector(31 downto 0);
        instr_id        : out    vl_logic_vector(31 downto 0);
        pc_id           : out    vl_logic_vector(31 downto 0)
    );
end iftoid;
