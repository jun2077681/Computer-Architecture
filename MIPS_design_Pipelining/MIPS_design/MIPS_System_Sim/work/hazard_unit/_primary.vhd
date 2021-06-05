library verilog;
use verilog.vl_types.all;
entity hazard_unit is
    port(
        regwrite_ma     : in     vl_logic;
        regwrite_wb     : in     vl_logic;
        rs_id           : in     vl_logic_vector(4 downto 0);
        rt_id           : in     vl_logic_vector(4 downto 0);
        rs_ex           : in     vl_logic_vector(4 downto 0);
        rt_ex           : in     vl_logic_vector(4 downto 0);
        writereg_ma     : in     vl_logic_vector(4 downto 0);
        writereg_wb     : in     vl_logic_vector(4 downto 0);
        forwardAE       : out    vl_logic_vector(1 downto 0);
        forwardBE       : out    vl_logic_vector(1 downto 0);
        memtoreg_ex     : in     vl_logic;
        hazard          : out    vl_logic;
        stall           : out    vl_logic;
        pcsrc           : in     vl_logic;
        jump_ex         : in     vl_logic;
        jrcontrol_ex    : in     vl_logic
    );
end hazard_unit;
