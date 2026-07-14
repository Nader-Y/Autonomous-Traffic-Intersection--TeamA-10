library ieee;
use ieee.std_logic_1164.all;

entity tb_intersection is
end entity;

architecture sim of tb_intersection is

signal clk        : std_logic := '0';
signal reset      : std_logic := '0';

signal req_id     : integer := 0;
signal start      : std_logic := '0';

signal grant      : std_logic;
signal conflict   : std_logic;
signal done       : std_logic;
signal quad_state : std_logic_vector(3 downto 0);

begin

------------------------------------------------------------
-- DUT
------------------------------------------------------------
uut: entity work.intersection_fpga
port map (
    clk => clk,
    reset => reset,
    req_id => req_id,
    start => start,
    grant => grant,
    conflict => conflict,
    quad_state => quad_state,
    done => done
);

------------------------------------------------------------
-- CLOCK
------------------------------------------------------------
clk_process : process
begin
    clk <= '0'; wait for 5 ns;
    clk <= '1'; wait for 5 ns;
end process;

------------------------------------------------------------
-- SINGLE SEQUENTIAL TEST PROCESS
------------------------------------------------------------
stim_proc : process
begin

------------------------------------------------------------
-- RESET TEST
------------------------------------------------------------
reset <= '1';
wait for 20 ns;
reset <= '0';

assert quad_state = "0000"
report "RESET FAILED"
severity error;

------------------------------------------------------------
-- TEST 1: VALID REQUEST
------------------------------------------------------------
req_id <= 1;
start <= '1';
wait for 10 ns;
start <= '0';

wait for 20 ns;

assert grant = '1'
report "TEST1 FAILED"
severity error;

------------------------------------------------------------
-- TEST 2: CONFLICT REQUEST
------------------------------------------------------------
req_id <= 3;
start <= '1';
wait for 10 ns;
start <= '0';

wait for 20 ns;

assert grant = '0'
report "TEST2 FAILED"
severity error;

------------------------------------------------------------
-- TEST 3: SEQUENTIAL EXECUTION
------------------------------------------------------------
req_id <= 2;
start <= '1';
wait for 10 ns;
start <= '0';

wait for 60 ns;

req_id <= 4;
start <= '1';
wait for 10 ns;
start <= '0';

wait for 20 ns;

assert done = '1'
report "TEST3 FAILED"
severity error;

------------------------------------------------------------
-- TEST 4: SLOT TIME RELEASE
------------------------------------------------------------
wait for 100 ns;

assert quad_state = "0000"
report "TEST4 FAILED"
severity error;

------------------------------------------------------------
-- TEST 5: INVALID INPUT
------------------------------------------------------------
req_id <= 0;
start <= '1';
wait for 10 ns;
start <= '0';

wait for 20 ns;

assert grant = '0'
report "TEST5 FAILED"
severity error;
------------------------------------------------------------
-- END
------------------------------------------------------------
report "ALL TESTS PASSED SUCCESSFULLY";
wait;

end process;

end architecture;