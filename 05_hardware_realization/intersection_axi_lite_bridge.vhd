library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity intersection_axi_lite_bridge is
    generic (
        -- Width of S_AXI data bus
        C_S_AXI_DATA_WIDTH : integer := 32;
        -- Width of S_AXI address bus (5 bits covers up to 0x1F)
        C_S_AXI_ADDR_WIDTH : integer := 5
    );
    port (
        -- Global Clock and Reset
        S_AXI_ACLK    : in std_logic;
        S_AXI_ARESETN : in std_logic; -- Active low reset

        -- Write Address Channel
        S_AXI_AWADDR  : in std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
        S_AXI_AWPROT  : in std_logic_vector(2 downto 0);
        S_AXI_AWVALID : in std_logic;
        S_AXI_AWREADY : out std_logic;

        -- Write Data Channel
        S_AXI_WDATA   : in std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
        S_AXI_WSTRB   : in std_logic_vector((C_S_AXI_DATA_WIDTH/8)-1 downto 0);
        S_AXI_WVALID  : in std_logic;
        S_AXI_WREADY  : out std_logic;

        -- Write Response Channel
        S_AXI_BRESP   : out std_logic_vector(1 downto 0);
        S_AXI_BVALID  : out std_logic;
        S_AXI_BREADY  : in std_logic;

        -- Read Address Channel
        S_AXI_ARADDR  : in std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
        S_AXI_ARPROT  : in std_logic_vector(2 downto 0);
        S_AXI_ARVALID : in std_logic;
        S_AXI_ARREADY : out std_logic;

        -- Read Data Channel
        S_AXI_RDATA   : out std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
        S_AXI_RRESP   : out std_logic_vector(1 downto 0);
        S_AXI_RVALID  : out std_logic;
        S_AXI_RREADY  : in std_logic
    );
end entity intersection_axi_lite_bridge;

architecture rtl of intersection_axi_lite_bridge is

    -- =========================================================
    -- AXI4-Lite Internal Signals
    -- =========================================================
    signal axi_awready : std_logic := '0';
    signal axi_wready  : std_logic := '0';
    signal axi_bresp   : std_logic_vector(1 downto 0) := "00";
    signal axi_bvalid  : std_logic := '0';
    signal axi_arready : std_logic := '0';
    signal axi_rdata   : std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0) := (others => '0');
    signal axi_rresp   : std_logic_vector(1 downto 0) := "00";
    signal axi_rvalid  : std_logic := '0';

    -- =========================================================
    -- Software Address Register Map (as expected by FreeRTOS)
    -- =========================================================
    -- 0x00 : CONTROL_REG    (Bit 0: start, Bit 7: emergency reset)
    -- 0x04 : REQUEST_ID_REG (Bits 3:0: Movement ID 1-12)
    -- 0x08 : STATUS_REG     (Bit 0: done, Bit 1: conflict)
    -- 0x0C : GRANT_REG      (Bit 0: grant)
    -- 0x10 : QUAD_REG       (Bits 3:0: Q4, Q3, Q2, Q1)
    
    signal slv_reg0_control : std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0) := (others => '0');
    signal slv_reg1_req_id  : std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0) := (others => '0');

    -- =========================================================
    -- Core Intersection Signals
    -- =========================================================
    signal core_reset      : std_logic;
    signal core_req_id     : integer range 0 to 12;
    signal core_start      : std_logic;
    signal core_grant      : std_logic;
    signal core_conflict   : std_logic;
    signal core_quad_state : std_logic_vector(3 downto 0);
    signal core_done       : std_logic;

    -- Local address decoding
    signal loc_addr : std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);

begin

    -- Assign AXI output pins
    S_AXI_AWREADY <= axi_awready;
    S_AXI_WREADY  <= axi_wready;
    S_AXI_BRESP   <= axi_bresp;
    S_AXI_BVALID  <= axi_bvalid;
    S_AXI_ARREADY <= axi_arready;
    S_AXI_RDATA   <= axi_rdata;
    S_AXI_RRESP   <= axi_rresp;
    S_AXI_RVALID  <= axi_rvalid;

    -- =========================================================
    -- INSTANTIATE INTERSECTION CORE
    -- =========================================================
    intersection_inst : entity work.intersection_fpga
        port map (
            clk        => S_AXI_ACLK,
            reset      => core_reset,
            req_id     => core_req_id,
            start      => core_start,
            grant      => core_grant,
            conflict   => core_conflict,
            quad_state => core_quad_state,
            done       => core_done
        );

    -- Map internal core signals to AXI registers
    core_reset  <= (not S_AXI_ARESETN) or slv_reg0_control(7); -- AXI reset or Software Emergency Reset
    core_start  <= slv_reg0_control(0);
    core_req_id <= to_integer(unsigned(slv_reg1_req_id(3 downto 0)));

    -- =========================================================
    -- AXI WRITE LOGIC (Processor -> FPGA)
    -- =========================================================
    process(S_AXI_ACLK)
    begin
        if rising_edge(S_AXI_ACLK) then
            if S_AXI_ARESETN = '0' then
                axi_awready <= '0';
                axi_wready  <= '0';
                axi_bvalid  <= '0';
                slv_reg0_control <= (others => '0');
                slv_reg1_req_id  <= (others => '0');
            else
                -- Accept Address
                if (axi_awready = '0' and S_AXI_AWVALID = '1' and S_AXI_WVALID = '1') then
                    axi_awready <= '1';
                    loc_addr <= S_AXI_AWADDR;
                else
                    axi_awready <= '0';
                end if;

                -- Accept Data
                if (axi_wready = '0' and S_AXI_WVALID = '1' and S_AXI_AWVALID = '1') then
                    axi_wready <= '1';
                else
                    axi_wready <= '0';
                end if;

                -- Write to Slave Registers
                if (axi_wready = '1' and S_AXI_WVALID = '1' and axi_awready = '1' and S_AXI_AWVALID = '1') then
                    case loc_addr is
                        when "00000" => -- 0x00: CONTROL_REG
                            slv_reg0_control <= S_AXI_WDATA;
                        when "00100" => -- 0x04: REQUEST_ID_REG
                            slv_reg1_req_id <= S_AXI_WDATA;
                        when others =>
                            -- 0x08, 0x0C, 0x10 are Read-Only for the processor
                            null; 
                    end case;
                end if;

                -- Send Write Response
                if (axi_awready = '1' and S_AXI_AWVALID = '1' and axi_wready = '1' and S_AXI_WVALID = '1' and axi_bvalid = '0') then
                    axi_bvalid <= '1';
                    axi_bresp  <= "00"; -- OKAY response
                elsif (S_AXI_BREADY = '1' and axi_bvalid = '1') then
                    axi_bvalid <= '0';
                end if;
            end if;
        end if;
    end process;

    -- =========================================================
    -- AXI READ LOGIC (FPGA -> Processor)
    -- =========================================================
    process(S_AXI_ACLK)
    begin
        if rising_edge(S_AXI_ACLK) then
            if S_AXI_ARESETN = '0' then
                axi_arready <= '0';
                axi_rvalid  <= '0';
                axi_rdata   <= (others => '0');
            else
                -- Accept Read Address
                if (axi_arready = '0' and S_AXI_ARVALID = '1') then
                    axi_arready <= '1';
                else
                    axi_arready <= '0';
                end if;

                -- Provide Read Data
                if (axi_arready = '1' and S_AXI_ARVALID = '1' and axi_rvalid = '0') then
                    axi_rvalid <= '1';
                    axi_rresp  <= "00"; -- OKAY response
                    
                    -- Multiplex the data out based on address
                    case S_AXI_ARADDR is
                        when "00000" => -- 0x00: CONTROL_REG
                            axi_rdata <= slv_reg0_control;
                            
                        when "00100" => -- 0x04: REQUEST_ID_REG
                            axi_rdata <= slv_reg1_req_id;
                            
                        when "01000" => -- 0x08: STATUS_REG
                            axi_rdata <= (others => '0');
                            axi_rdata(1) <= core_conflict;
                            axi_rdata(0) <= core_done;
                            
                        when "01100" => -- 0x0C: GRANT_REG
                            axi_rdata <= (others => '0');
                            axi_rdata(0) <= core_grant;
                            
                        when "10000" => -- 0x10: QUAD_REG
                            axi_rdata <= (others => '0');
                            axi_rdata(3 downto 0) <= core_quad_state;
                            
                        when others =>
                            axi_rdata <= (others => '0');
                    end case;
                elsif (axi_rvalid = '1' and S_AXI_RREADY = '1') then
                    axi_rvalid <= '0';
                end if;
            end if;
        end if;
    end process;

end architecture;