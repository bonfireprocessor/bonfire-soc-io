----------------------------------------------------------------------------------

-- Module Name:    bonfire_soc_io - Behavioral

-- The Bonfire Processor Project, (c) 2016,2017 Thomas Hornschuh

--  SOC IO Block with Wishbone interface
--  Currently supports: 2* UART, 1* GPIO, 1* SPI Flash


-- License: See LICENSE or LICENSE.txt File in git project root.
----------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;

use work.txt_util.all;


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
   ENABLE_GPIO : boolean := true;

   ENABLE_SIRC : boolean := false;
   SIRC_IRQS : natural := 12;
   UART0_IRC_NUM : natural := 1;
   UART1_IRC_NUM : natural := 2;
   GPIO_RISE_IRC_NUM: natural :=3;
   GPIO_FALL_IRC_NUM: natural :=4;
   GPIO_HIGH_IRC_NUM: natural :=5;
   GPIO_LOW_IRC_NUM: natural :=6;
   ADITIONAL_SIRC_IRQ_LOW : natural := 7;


   IRQ_LEGACY_MODE : boolean := true -- Use old IRQ mechanism in parallel

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


       irq_o: out std_logic_vector(7 downto 0); -- old style "local" irqs
       sirc_irq_o : out std_logic;   -- sirc irq request out
       sirc_irq_i : in std_logic_vector(SIRC_IRQS downto ADITIONAL_SIRC_IRQ_LOW);


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

constant slaves : natural := 5;

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

-- IRQs
signal uart0_irq, uart1_irq,
       gpio_fall_irq,gpio_high_irq,
       gpio_low_irq, gpio_rise_irq : std_logic;

signal sirc_irq_req : std_logic_vector(SIRC_IRQS downto 1);


function valid_irqnum(i : natural) return boolean is
begin
    return i >=1 and i <= SIRC_IRQS;
end function;

function check_sirq_consistency return boolean is
type t_irq_assignments is array (sirc_irq_req'range) of boolean;
variable irq_assignments : t_irq_assignments := (others=>false);


   function assign_irq(n : natural) return boolean is
   begin
      if valid_irqnum(n) and  not irq_assignments(n) then
        irq_assignments(n) := true;
        return true;
      else
        return false;
      end if;
   end function;

   begin
      assert assign_irq(UART0_IRC_NUM) report "assignment conflict with UART0_IRC_NUM :=" & str(UART0_IRC_NUM) severity failure;
      assert assign_irq(UART1_IRC_NUM) report "assignment conflict with UART1_IRC_NUM :=" & str(UART1_IRC_NUM) severity failure;
      assert assign_irq(GPIO_RISE_IRC_NUM) report "assignment conflict with GPIO_RISE_IRC_NUM :=" & str(GPIO_RISE_IRC_NUM) severity failure;
      assert assign_irq(GPIO_FALL_IRC_NUM) report "assignment conflict with GPIO_FALL_IRC_NUM :=" & str(GPIO_FALL_IRC_NUM) severity failure;
      assert assign_irq(GPIO_HIGH_IRC_NUM) report "assignment conflict with GPIO_HIGH_IRC_NUM :=" & str(GPIO_HIGH_IRC_NUM) severity failure;
      assert assign_irq(GPIO_LOW_IRC_NUM) report "assignment conflict with GPIO_LOW_IRC_NUM :=" & str(GPIO_LOW_IRC_NUM) severity failure;

      assert ADITIONAL_SIRC_IRQ_LOW<=SIRC_IRQS report "ADITIONAL_SIRC_IRQ_LOW out of range" severity failure;
      for i in ADITIONAL_SIRC_IRQ_LOW to SIRC_IRQS loop
         assert assign_irq(i)  report "assignment conflict with SIRQ(" & str(i) & ")" severity failure;
      end loop;
      return  true;

end function;


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

assert ENABLE_SIRC and check_sirq_consistency
      report "SIRC IRQ consitency check failed" severity failure;

g_irq_legacy: if IRQ_LEGACY_MODE  generate

    irq_o(7) <= uart0_irq;
    irq_o(6) <= uart1_irq;
    irq_o(5 downto 0) <= (others=>'0');

else generate

    irq_o(7 downto 0) <= (others=>'0');

end generate;


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
        m3_dat_i => m_dat_i(3),

        m4_cyc_o => m_cyc_o(4) ,
        m4_stb_o => m_stb_o(4),
        m4_we_o => m_we_o(4),
        m4_sel_o =>m_sel_o(4) ,
        m4_ack_i => m_ack_i(4),
        m4_adr_o => m_adr_o(4),
        m4_dat_o => m_dat_o(4),
        m4_dat_i => m_dat_i(4)
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
            wb_inta_o => uart0_irq,  -- irq_o(irq_o'high) ,
            id => open,
            enabled => open,
            tx => uart0_txd,
            rx => uart0_rxd
        );

end generate;



g_no_uart_0: if not ENABLE_UART0 generate
   m_ack_i(0) <= m_cyc_o(0) and  m_stb_o(0);
   uart0_irq <= '0';
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
            wb_inta_o => uart1_irq, --irq_o(irq_o'high-1) ,
            id => open,
            enabled => open,
            tx => uart1_txd,
            rx => uart1_rxd
        );


end generate;

g_no_uart_1: if not ENABLE_UART1 generate
   m_ack_i(2) <= m_cyc_o(2) and  m_stb_o(2);
   uart1_irq <= '0';
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

            rise_irq_o => gpio_rise_irq,
            fall_irq_o => gpio_fall_irq,
            high_irq_o => gpio_high_irq,
            low_irq_o => gpio_low_irq
        );

end generate;


g_no_gpio: if not ENABLE_GPIO generate
   m_ack_i(3) <= m_cyc_o(3) and  m_stb_o(3);
   gpio_fall_irq <= '0';
   gpio_rise_irq <= '0';
   gpio_high_irq <= '0';
   gpio_low_irq  <= '0';
end generate;


g_sirc: if ENABLE_SIRC generate

    Inst_sirc: entity work.simple_irc
    GENERIC MAP (
        NUM_IRQ => SIRC_IRQS
    )

    PORT MAP(

            clk_i => clk_i,
            rst_i => rst_i,
            wbs_cyc_i => m_cyc_o(4),
            wbs_stb_i => m_stb_o(4),
            wbs_we_i => m_we_o(4),

            wbs_ack_o => m_ack_i(4),
            wbs_adr_i => m_adr_o(4)(3 downto 2),
            wbs_dat_i => m_dat_o(4),
            wbs_dat_o => m_dat_i(4),

            irq_in => sirc_irq_req,
            irq_req_o => sirc_irq_o

        );


        process(all)
        begin
          sirc_irq_req <= (others => '0');
          if valid_irqnum(UART0_IRC_NUM) then
             sirc_irq_req(UART0_IRC_NUM) <= uart0_irq;
          end if;
          if valid_irqnum(UART1_IRC_NUM) then
             sirc_irq_req(UART1_IRC_NUM) <= uart1_irq;
          end if;
          if valid_irqnum(GPIO_RISE_IRC_NUM) then
             sirc_irq_req(GPIO_RISE_IRC_NUM) <= gpio_rise_irq;
          end if;
          if valid_irqnum(GPIO_FALL_IRC_NUM) then
             sirc_irq_req(GPIO_FALL_IRC_NUM) <= gpio_fall_irq;
          end if;
          if valid_irqnum(GPIO_HIGH_IRC_NUM) then
             sirc_irq_req(GPIO_HIGH_IRC_NUM) <= gpio_high_irq;
          end if;
          if valid_irqnum(GPIO_LOW_IRC_NUM) then
             sirc_irq_req(GPIO_LOW_IRC_NUM) <= gpio_low_irq;
          end if;

        end process;

 else generate
    sirc_irq_o <= '0';
    m_ack_i(4) <= m_cyc_o(4) and  m_stb_o(4);

 end generate;



end architecture;