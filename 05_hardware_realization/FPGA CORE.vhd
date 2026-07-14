library ieee;
use ieee.std_logic_1164.all;

entity intersection_fpga is
port (
    clk        : in  std_logic;
    reset      : in  std_logic;

    req_id     : in  integer range 0 to 12;
    start      : in  std_logic;

    grant      : out std_logic;
    conflict   : out std_logic;
    quad_state : out std_logic_vector(3 downto 0);
    done       : out std_logic
);
end entity;

architecture rtl of intersection_fpga is

    signal req_path     : std_logic_vector(3 downto 0) := "0000";
    signal occupied     : std_logic_vector(3 downto 0) := "0000";
    signal timer        : integer := 0;

    signal reserve      : std_logic := '0';
    signal release      : std_logic := '0';
    signal conflict_int : std_logic := '0';

begin

process(req_id)
begin
    case req_id is
        when 1  => req_path <= "0011";
        when 2  => req_path <= "1100";
        when 3  => req_path <= "1100";
        when 4  => req_path <= "0011";

        when 5  => req_path <= "1001";
        when 6  => req_path <= "1100";
        when 7  => req_path <= "1100";
        when 8  => req_path <= "0011";

        when 9  => req_path <= "0011";
        when 10 => req_path <= "0011";
        when 11 => req_path <= "1100";
        when 12 => req_path <= "1100";

        when others => req_path <= "0000";
    end case;
end process;

conflict_int <= '1' when ((occupied and req_path) /= "0000") else '0';
conflict <= conflict_int;

process(clk, reset)
begin
    if reset = '1' then
        grant   <= '0';
        reserve <= '0';

    elsif rising_edge(clk) then
        if start = '1' then
            if conflict_int = '0' then
                grant   <= '1';
                reserve <= '1';
            else
                grant   <= '0';
                reserve <= '0';
            end if;
        else
            grant   <= '0';
            reserve <= '0';
        end if;
    end if;
end process;

process(clk, reset)
begin
    if reset = '1' then
        occupied   <= "0000";
        quad_state <= "0000";

    elsif rising_edge(clk) then
        if reserve = '1' then
            occupied <= occupied or req_path;

        elsif release = '1' then
            occupied <= "0000";
        end if;

        quad_state <= occupied;
    end if;
end process;

process(clk, reset)
begin
    if reset = '1' then
        timer   <= 0;
        release <= '0';
        done    <= '0';

    elsif rising_edge(clk) then
        if reserve = '1' then
            if timer = 5 then
                release <= '1';
                done    <= '1';
                timer   <= 0;
            else
                timer   <= timer + 1;
                release <= '0';
                done    <= '0';
            end if;
        else
            timer   <= 0;
            release <= '0';
            done    <= '0';
        end if;
    end if;
end process;

end architecture;