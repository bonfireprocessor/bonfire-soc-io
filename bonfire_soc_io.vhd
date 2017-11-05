

library ieee;
use ieee.std_logic_1164.all;

entity bonfire_soc_io is
generic (
   NUM_GPIO_BITS : natural := 32;
   ADR_HIGH : natural := 25;
   UART_FIFO_DEPTH : natural := 6 -- log2 of  UART fifo depth
);
Port(


       -- UART0 signals:
       uart0_txd : out std_logic;
       uart0_rxd : in  std_logic :='1';

        -- UART1 signals:
       uart1_txd : out std_logic;
       uart1_rxd : in  std_logic :='1';

       -- GPIO
       gpio_o : out std_logic_vector(NUM_GPIO_BITS downto 0);
       gpio_i : in  std_logic_vector(NUM_GPIO_BITS downto 0);
       gpio_t : out std_logic_vector(NUM_GPIO_BITS downto 0);


       -- SPI flash chip
       flash_spi_cs        : out   std_logic;
       flash_spi_clk       : out   std_logic;
       flash_spi_mosi      : out   std_logic;
       flash_spi_miso      : in    std_logic;


       irq_o: out std_logic_vector(7 downto 0);


       -- Wishbone Slave
       clk_i: in std_logic;
       rst_i: in std_logic;

       wb_cyc_i: in std_logic;
       wb_stb_i: in std_logic;
       wb_we_i: in std_logic;
       wb_sel_i : in std_logic_vector(3 downto 0);
       wb_ack_o: out std_logic;
       wb_adr_i: in std_logic_vector(ADR_HIGH downto 2); -- only bits 29 downto 2 are used !
       wb_dat_i: in std_logic_vector(31 downto 0);
       wb_dat_o: out std_logic_vector(31 downto 0)

);
end bonfire_soc_io;

architecture rtl of bonfire_soc_io is

constant slaves : natural := 4;

subtype t_wbdat is  std_logic_vector(wb_dat_i'range);
subtype t_wbadr is  std_logic_vector(15 downto 2);
subtype t_wbsel is  std_logic_vector(wb_sel_i'range);

type t_wbdat_a is array(natural range <>) of t_wbdat;
type t_wbadr_a is array(natural range <>) of t_wbadr;
type t_wbsel_a is array(natural range <>) of t_wbsel;

signal m_cyc_o :  std_logic_vector(0 to slaves-1);
signal m_stb_o :  std_logic_vector(0 to slaves-1);
signal m_we_o :  std_logic_vector(0 to slaves-1);
signal m_dat_o :  t_wbdat_a(0 to slaves-1);
signal m_dat_i :  t_wbdat_a(0 to slaves-1);
signal m_adr_o :  t_wbadr_a(0 to slaves-1);
signal m_sel_o :  t_wbsel_a(0 to slaves-1);
signal m_ack_i :  std_logic_vector(0 to slaves-1);

begin

Inst_io_intercon: entity work.io_intercon PORT MAP(
        clk_i => clk_i,
        rst_i => rst_i,
        s0_cyc_i => wb_cyc_i,
        s0_stb_i => wb_stb_i,
        s0_we_i =>  wb_we_i,
        s0_sel_i => wb_sel_i,
        s0_ack_o => wb_ack_o,
        s0_adr_i => wb_adr_i,
        s0_dat_i => wb_dat_i,
        s0_dat_o => wb_dat_o,

        m0_cyc_o => m_cyc_o(0) ,
        m0_stb_o => m_stb_o(0),
        m0_we_o => m_we_o(0),
        m0_sel_o =>m_sel_o(0) ,
        m0_ack_i => m_ack_i(0),
        m0_adr_o => m_adr_o(0),
        m0_dat_o => m_dat_o(0),
        m0_dat_i => m_dat_i(0),
        m1_cyc_o => m_cyc_o(1) ,
        m1_stb_o => m_stb_o(1),
        m1_we_o => m_we_o(1),
        m1_sel_o =>m_sel_o(1) ,
        m1_ack_i => m_ack_i(1),
        m1_adr_o => m_adr_o(1),
        m1_dat_o => m_dat_o(1),
        m1_dat_i => m_dat_i(1),
        m2_cyc_o => m_cyc_o(2) ,
        m2_stb_o => m_stb_o(2),
        m2_we_o => m_we_o(2),
        m2_sel_o =>m_sel_o(2) ,
        m2_ack_i => m_ack_i(2),
        m2_adr_o => m_adr_o(2),
        m2_dat_o => m_dat_o(2),
        m2_dat_i => m_dat_i(2),
        m3_cyc_o => m_cyc_o(3) ,
        m3_stb_o => m_stb_o(3),
        m3_we_o => m_we_o(3),
        m3_sel_o =>m_sel_o(3) ,
        m3_ack_i => m_ack_i(3),
        m3_adr_o => m_adr_o(3),
        m3_dat_o => m_dat_o(3),
        m3_dat_i => m_dat_i(3)
    );


uart_0: entity work.zpuino_uart
GENERIC MAP (
  bits => UART_FIFO_DEPTH
)

PORT MAP(
        wb_clk_i => clk_i,
        wb_rst_i => rst_i,
        wb_dat_o =>  m_dat_i(0),
        wb_dat_i =>  m_dat_o(0),
        wb_adr_i =>  m_adr_o(0)(2 downto 2),
        wb_we_i =>   m_we_o(0),
        wb_cyc_i =>  m_cyc_o(0),
        wb_stb_i =>  m_stb_o(0),
        wb_ack_o =>  m_ack_i(0),
        wb_inta_o => open ,
        id => open,
        enabled => open,
        tx => uart0_txd,
        rx => uart0_rxd
    );


uart_1: entity work.zpuino_uart
GENERIC MAP (
  bits => UART_FIFO_DEPTH 
)

PORT MAP(
        wb_clk_i => clk_i,
        wb_rst_i => rst_i,
        wb_dat_o =>  m_dat_i(1),
        wb_dat_i =>  m_dat_o(1),
        wb_adr_i =>  m_adr_o(1)(2 downto 2),
        wb_we_i =>   m_we_o(1),
        wb_cyc_i =>  m_cyc_o(1),
        wb_stb_i =>  m_stb_o(1),
        wb_ack_o =>  m_ack_i(1),
        wb_inta_o => open ,
        id => open,
        enabled => open,
        tx => uart1_txd,
        rx => uart1_rxd
    );


end architecture;
