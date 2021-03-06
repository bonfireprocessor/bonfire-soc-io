----------------------------------------------------------------------------------

-- Module Name:    bonfire_soc_io - Behavioral

-- The Bonfire Processor Project, (c) 2016,2017 Thomas Hornschuh

--  SOC IO Block with Wishbone interface
--  Currently supports: 2* UART, 1* GPIO, 1* SPI Flash


-- License: See LICENSE or LICENSE.txt File in git project root.
----------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;

entity bonfire_soc_io is
generic (
   CLK_FREQUENCY : natural := (96 * 1000000);
   NUM_GPIO_BITS : natural := 32;
   ADR_HIGH : natural := 25;
   UART_FIFO_DEPTH : natural := 6; -- log2 of  UART fifo depth
   ENABLE_UART0 : boolean := true;
   ENABLE_UART1 : boolean := true;
   ENABLE_SPI : boolean := true;
   NUM_SPI : natural := 1;
   ENABLE_GPIO : boolean := true
);
Port(


       -- UART0 signals:
       uart0_txd : out std_logic;
       uart0_rxd : in  std_logic :='1';

        -- UART1 signals:
       uart1_txd : out std_logic;
       uart1_rxd : in  std_logic :='1';

       -- GPIO
       gpio_o : out std_logic_vector(NUM_GPIO_BITS-1 downto 0);
       gpio_i : in  std_logic_vector(NUM_GPIO_BITS-1 downto 0);
       gpio_t : out std_logic_vector(NUM_GPIO_BITS-1 downto 0);


       -- SPI ports
       spi_cs        : out   std_logic_vector(NUM_SPI-1 downto 0);
       spi_clk       : out   std_logic_vector(NUM_SPI-1 downto 0);
       spi_mosi      : out   std_logic_vector(NUM_SPI-1 downto 0);
       spi_miso      : in    std_logic_vector(NUM_SPI-1 downto 0);


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

subtype t_wbdat is  std_logic_vector(wb_dat_i'high downto wb_dat_i'low);
subtype t_wbadr is  std_logic_vector(15 downto 2);
subtype t_wbsel is  std_logic_vector(wb_sel_i'high downto wb_sel_i'low);

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


component bonfire_spi
    generic (

       CPOL : std_logic := '0';  -- SPI mode selection (mode 0 default)
       CPHA : std_logic := '0';  -- CPOL = clock polarity, CPHA = clock phase.
       SPI_2X_CLK_DIV : natural := 2;


       WB_DATA_WIDTH : natural :=32;
       ADR_LOW  : natural :=2;
       NUM_PORTS : natural := 1
    );
    port (
          spi_clk_i : in std_logic;


          -- SPI Port:
          slave_cs_o         : out std_logic_vector(NUM_PORTS-1 downto 0);
          slave_clk_o        : out std_logic_vector(NUM_PORTS-1 downto 0);
          slave_mosi_o       : out std_logic_vector(NUM_PORTS-1 downto 0);
          slave_miso_i       : in  std_logic_vector(NUM_PORTS-1 downto 0);

          -- Interrupt signal:
          irq : out std_logic;

          -- Wishbone ports:
          wb_clk_i   : in std_logic;
          wb_rst_i   : in std_logic;
          wb_adr_in  : in  std_logic_vector(ADR_LOW+7 downto ADR_LOW);
          wb_dat_in  : in  std_logic_vector(WB_DATA_WIDTH-1 downto 0);
          wb_dat_out : out std_logic_vector(WB_DATA_WIDTH-1 downto 0);
          wb_we_in   : in  std_logic;
          wb_cyc_in  : in  std_logic;
          wb_stb_in  : in  std_logic;
          wb_ack_out : out std_logic
    );
end component bonfire_spi;


begin

irq_o(irq_o'high-6 downto 0) <= (others=>'0'); -- temporary


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



g_uart0: if ENABLE_UART0 generate

    uart_0: entity work.zpuino_uart
    GENERIC MAP (
    bits => UART_FIFO_DEPTH,
    extended => true
    )

    PORT MAP(
            wb_clk_i => clk_i,
            wb_rst_i => rst_i,
            wb_dat_o =>  m_dat_i(0),
            wb_dat_i =>  m_dat_o(0),
            wb_adr_i =>  m_adr_o(0)(3 downto 2),
            wb_we_i =>   m_we_o(0),
            wb_cyc_i =>  m_cyc_o(0),
            wb_stb_i =>  m_stb_o(0),
            wb_ack_o =>  m_ack_i(0),
            wb_inta_o => irq_o(irq_o'high) ,
            id => open,
            enabled => open,
            tx => uart0_txd,
            rx => uart0_rxd
        );

end generate;

g_no_uart_0: if not ENABLE_UART0 generate
   m_ack_i(0) <= m_cyc_o(0) and  m_stb_o(0);
   irq_o(irq_o'high) <= '0';
end generate;


g_uart1: if ENABLE_UART1 generate

    uart_1: entity work.zpuino_uart
    GENERIC MAP (
    bits => UART_FIFO_DEPTH,
    extended => true
    )

    PORT MAP(
            wb_clk_i => clk_i,
            wb_rst_i => rst_i,
            wb_dat_o =>  m_dat_i(2),
            wb_dat_i =>  m_dat_o(2),
            wb_adr_i =>  m_adr_o(2)(3 downto 2),
            wb_we_i =>   m_we_o(2),
            wb_cyc_i =>  m_cyc_o(2),
            wb_stb_i =>  m_stb_o(2),
            wb_ack_o =>  m_ack_i(2),
            wb_inta_o => irq_o(irq_o'high-1) ,
            id => open,
            enabled => open,
            tx => uart1_txd,
            rx => uart1_rxd
        );


end generate;

g_no_uart_1: if not ENABLE_UART1 generate
   m_ack_i(2) <= m_cyc_o(2) and  m_stb_o(2);
   irq_o(irq_o'high-1) <= '0';
end generate;

g_spi: if ENABLE_SPI generate

    spi_flash: bonfire_spi
    generic map (
      NUM_PORTS => NUM_SPI
    )
    PORT MAP(
            wb_clk_i => clk_i,
            wb_rst_i => rst_i,
            spi_clk_i => clk_i,

            slave_cs_o => spi_cs,
            slave_clk_o => spi_clk,
            slave_mosi_o => spi_mosi,
            slave_miso_i => spi_miso,
            irq => open,
            wb_adr_in => m_adr_o(1)(9 downto 2),
            wb_dat_in => m_dat_o(1),
            wb_dat_out => m_dat_i(1),
            wb_we_in =>  m_we_o(1),
            wb_cyc_in => m_cyc_o(1),
            wb_stb_in => m_stb_o(1),
            wb_ack_out =>m_ack_i(1)
        );

end generate;

g_no_spi: if not ENABLE_SPI generate
   m_ack_i(1) <= m_cyc_o(1) and  m_stb_o(1);
end generate;

g_gpio: if ENABLE_GPIO generate

    Inst_gpio: entity work.bonfire_gpio
    GENERIC MAP (
    maxIObit => t_wbadr'high,
    NUM_GPIO_BITS => gpio_o'length
    )

    PORT MAP(
            gpio_o =>gpio_o,
            gpio_i =>gpio_i,
            gpio_t =>gpio_t,

            wb_clk_i => clk_i,
            wb_rst_i => rst_i,
            wb_cyc_i => m_cyc_o(3),
            wb_stb_i => m_stb_o(3),
            wb_we_i => m_we_o(3),

            wb_ack_o => m_ack_i(3),
            wb_adr_i => m_adr_o(3),
            wb_dat_i => m_dat_o(3),
            wb_dat_o => m_dat_i(3),

            rise_irq_o => irq_o(irq_o'high-2),
            fall_irq_o => irq_o(irq_o'high-3),
            high_irq_o => irq_o(irq_o'high-4),
            low_irq_o => irq_o(irq_o'high-5)
        );

end generate;

g_no_gpio: if not ENABLE_GPIO generate
   m_ack_i(3) <= m_cyc_o(3) and  m_stb_o(3);
   irq_o(irq_o'high-2 downto irq_o'high-5) <= (others =>'0');
end generate;

end architecture;