--------------------------------------------------------------------------------
-- Company:
-- Engineer:
--
-- Create Date:   21:14:18 11/06/2017
-- Design Name:
-- Module Name:   tb_bonfire_soc_io

-- The Bonfire Processor Project, (c) 2016,2017 Thomas Hornschuh

--  Test Bench for bonfire_soc_io
--  This test bench does not toroughly test the enclosed I/O cores.
--  It is more intended to check that all modules are wired correctly and can be
--  addressed. Unit testing on the I/O cores should be done with their test benches



-- License: See LICENSE or LICENSE.txt File in git project root.
--------------------------------------------------------------------------------
LIBRARY ieee;
USE ieee.std_logic_1164.ALL;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
USE ieee.numeric_std.ALL;

LIBRARY std;
USE std.textio.all;

use work.txt_util.all;



ENTITY tb_bonfire_soc_io IS
  generic (
    SKIP_UART_TEST : boolean := false
  );
END tb_bonfire_soc_io;

ARCHITECTURE behavior OF tb_bonfire_soc_io IS

    -- Component Declaration for the Unit Under Test (UUT)

    COMPONENT bonfire_soc_io
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
       SIRC_IRQS : natural := 8;
       UART0_IRC_NUM : natural := 1;
       UART1_IRC_NUM : natural := 2;
       GPIO_RISE_IRC_NUM: natural :=3;
       GPIO_FALL_IRC_NUM: natural :=4;
       GPIO_HIGH_IRC_NUM: natural :=5;
       GPIO_LOW_IRC_NUM: natural :=6;
       ADITIONAL_SIRC_IRQ_LOW : natural := 7;

       IRQ_LEGACY_MODE : boolean := true -- Use old IRQ mechanism in parallel
    );

    PORT(
         uart0_txd : OUT  std_logic;
         uart0_rxd : IN  std_logic;
         uart1_txd : OUT  std_logic;
         uart1_rxd : IN  std_logic;
         gpio_o : out std_logic_vector(NUM_GPIO_BITS-1 downto 0);
         gpio_i : in  std_logic_vector(NUM_GPIO_BITS-1 downto 0);
         gpio_t : out std_logic_vector(NUM_GPIO_BITS-1 downto 0);
         spi_cs        : out   std_logic_vector(NUM_SPI-1 downto 0);
         spi_clk       : out   std_logic_vector(NUM_SPI-1 downto 0);
         spi_mosi      : out   std_logic_vector(NUM_SPI-1 downto 0);
         spi_miso      : in    std_logic_vector(NUM_SPI-1 downto 0);
         irq_o : OUT  std_logic_vector(7 downto 0);
         sirc_irq_o : out std_logic;

         clk_i : IN  std_logic;
         rst_i : IN  std_logic;
         wb_cyc_i : IN  std_logic;
         wb_stb_i : IN  std_logic;
         wb_we_i : IN  std_logic;
         wb_sel_i : IN  std_logic_vector(3 downto 0);
         wb_ack_o : OUT  std_logic;
         wb_adr_i : IN  std_logic_vector(ADR_HIGH downto 2);
         wb_dat_i : IN  std_logic_vector(31 downto 0);
         wb_dat_o : OUT  std_logic_vector(31 downto 0)
        );
    END COMPONENT;


    COMPONENT tb_uart_capture_tx
    GENERIC (
      baudrate : natural;
      bit_time : time;
      SEND_LOG_NAME : string ;
      stop_mark : std_logic_vector(7 downto 0) -- Stop marker byte
     );
    PORT(
        txd : IN std_logic;
        stop : OUT boolean;
        framing_errors : OUT natural;
        total_count : OUT natural
        );
    END COMPONENT;



   --Inputs
   signal uart0_rxd : std_logic := '0';
   signal uart1_rxd : std_logic := '0';
   signal gpio_i : std_logic_vector(31 downto 0) := (others => '0');
   signal flash_spi_miso : std_logic := '0';
   signal clk_i : std_logic := '0';
   signal rst_i : std_logic := '0';
   signal wb_cyc_i : std_logic := '0';
   signal wb_stb_i : std_logic := '0';
   signal wb_we_i : std_logic := '0';
   signal wb_sel_i : std_logic_vector(3 downto 0) := (others => '0');
   signal wb_adr_i : std_logic_vector(25 downto 2) := (others => '0');
   signal wb_dat_i : std_logic_vector(31 downto 0) := (others => '0');

    --Outputs
   signal uart0_txd : std_logic;
   signal uart1_txd : std_logic;
   signal gpio_o : std_logic_vector(31 downto 0);
   signal gpio_t : std_logic_vector(31 downto 0);
   signal flash_spi_cs : std_logic;
   signal flash_spi_clk : std_logic;
   signal flash_spi_mosi : std_logic;
   signal irq_o : std_logic_vector(7 downto 0);
   signal wb_ack_o : std_logic;
   signal wb_dat_o : std_logic_vector(31 downto 0);
   signal sirq_request : std_logic;

   -- Clock period definitions

   constant clk_i_period : time := 10.41  ns;  --Clock 96Mhz

   signal TbSimEnded : std_logic := '0';


   constant io_offset : natural :=2**16;

   subtype t_adr_s is signed(31 downto 0);

   constant UART_0_BASE : t_adr_s :=(others=>'0');
   constant FLASH_SPI_BASE : t_adr_s := UART_0_BASE+io_offset;
   constant UART_1_BASE : t_adr_s := FLASH_SPI_BASE+io_offset;
   constant GPIO_BASE : t_adr_s := UART_1_BASE+io_offset;
   constant SIRC_BASE : t_adr_s := GPIO_BASE+io_offset;

   signal uart0_stop,uart1_stop : boolean;

   subtype t_uartnum is natural range 0 to 1;
   type t_uart_kpi is array (t_uartnum) of natural;

   signal total_count : t_uart_kpi;
   signal framing_errors : t_uart_kpi;

   constant baudrate : natural := 115200;
   constant bit_time : time := 8.68 us;


BEGIN

    -- Instantiate the Unit Under Test (UUT)
   uut: bonfire_soc_io
   GENERIC MAP (
    ENABLE_SIRC => TRUE
   )

   PORT MAP (
          uart0_txd => uart0_txd,
          uart0_rxd => uart0_rxd,
          uart1_txd => uart1_txd,
          uart1_rxd => uart1_rxd,
          gpio_o => gpio_o,
          gpio_i => gpio_i,
          gpio_t => gpio_t,
          spi_cs(0) => flash_spi_cs,
          spi_clk(0) => flash_spi_clk,
          spi_mosi(0) => flash_spi_mosi,
          spi_miso(0) => flash_spi_miso,
          irq_o => irq_o,
          sirc_irq_o => sirq_request,
          clk_i => clk_i,
          rst_i => rst_i,
          wb_cyc_i => wb_cyc_i,
          wb_stb_i => wb_stb_i,
          wb_we_i => wb_we_i,
          wb_sel_i => wb_sel_i,
          wb_ack_o => wb_ack_o,
          wb_adr_i => wb_adr_i,
          wb_dat_i => wb_dat_i,
          wb_dat_o => wb_dat_o
        );


   capture_tx_0 :  tb_uart_capture_tx
   GENERIC MAP (
       baudrate => baudrate,
       bit_time => bit_time,
       SEND_LOG_NAME => "send0.log",
       stop_mark => X"1A"
   )
   PORT MAP(
        txd => uart0_txd,
        stop => uart0_stop ,
        framing_errors => framing_errors(0),
        total_count =>total_count(0)
    );


    capture_tx_1 :  tb_uart_capture_tx
    GENERIC MAP (
       baudrate => baudrate,
       bit_time => bit_time,
       SEND_LOG_NAME => "send1.log",
       stop_mark => X"1A"
    )
    PORT MAP(
        txd => uart1_txd,
        stop => uart1_stop ,
        framing_errors => framing_errors(1),
        total_count =>total_count(1)
    );





   clk_i <= not clk_i after clk_i_period/2 when TbSimEnded /= '1' else '0';

   flash_spi_miso <= flash_spi_mosi; -- loop back

   -- Stimulus process
   stim_proc: process
       variable d,t : std_logic_vector(wb_dat_i'range);


       procedure wb_write(address : in t_adr_s; data : in std_logic_vector(wb_dat_i'range)) is
         begin
            wait until rising_edge(clk_i);
            wb_adr_i <= std_logic_vector(address(wb_adr_i'range));
            wb_dat_i <= data;
            wb_we_i <= '1';
            wb_cyc_i <= '1';
            wb_stb_i <= '1';
            wb_sel_i <="1111";

            wait  until rising_edge(clk_i) and wb_ack_o = '1' ;
            wb_stb_i <= '0';
            wb_cyc_i <= '0';
            wb_we_i <=  '0';

        end procedure;

       procedure wb_read(address : in t_adr_s;
                          data: out std_logic_vector(wb_dat_o'range) )  is
         begin
            wait until rising_edge(clk_i);
            wb_adr_i <= std_logic_vector(address(wb_adr_i'range));
            wb_we_i <= '1';
            wb_cyc_i <= '1';
            wb_stb_i <= '1';
            wb_we_i <= '0';
            wb_sel_i <="1111";
            wait until rising_edge(clk_i) and wb_ack_o = '1';
            data:= wb_dat_o;
            wb_stb_i <= '0';
            wb_cyc_i <= '0';

        end procedure;


        procedure test_spi_loopback is
        begin
          print(OUTPUT,"Testing SPI Interface");
          wb_write(FLASH_SPI_BASE+16,X"00000001"); -- Clock Divider
          wb_write(FLASH_SPI_BASE,X"FFFFFFFE"); -- Chip Select
          -- send 10 bytes
          for i in 0 to 255 loop
            t:=std_logic_vector(to_unsigned(i,t'length));
            wb_write(FLASH_SPI_BASE+8,t);
            wb_read(FLASH_SPI_BASE+12,d);
            if d(7 downto 0) /= t(7 downto 0) then
              report "SPI Interface test Failure";
              wait;
            end if;

          end loop;
          print(OUTPUT,"SPI Interface ok");
        end procedure;

        procedure uart_tx(uart:t_uartnum; byte:t_byte) is
        variable status : std_logic_vector(31 downto 0);
        variable adr_base : t_adr_s;
        begin
           case uart is
             when 0 =>
               adr_base:=UART_0_BASE;
             when 1 =>
               adr_base:=UART_1_BASE;
           end case;

           status:=(others=>'U');
           while status(1)/='1'  loop
             wb_read(adr_base+4,status);
           end loop;
           wb_write(adr_base,X"000000" & byte);
        end;


        procedure check_sirq_claim(v: std_logic_vector(wb_dat_i'range)) is
        variable d : std_logic_vector(wb_dat_i'range);
        begin
          wb_read(SIRC_BASE+8,d);
          print(OUTPUT,"Interrupt claim register:" & hstr(d));
          assert d=v report "Invalid value for interrupt claim register" severity failure;
        end procedure;

        procedure sirc_test is
        constant irq3_mask : std_logic_vector(wb_dat_i'range) := (3=>'1',others=>'0');
        begin
          print(OUTPUT,"Testing SIRC IE Register");
          wb_read(SIRC_BASE+4,d);
          assert d= 32b"0" report "IE register reset value invalid" severity failure ;
          assert sirq_request='0' report "sirq_request is not 0 initially" severity failure;
          check_sirq_claim(32ub"0");
          wb_write(SIRC_BASE+4,X"FFFFFFFF");
          wb_read(SIRC_BASE+4,d);
          print(OUTPUT,"IE Register read: " & str(d));
          assert d=32ub"111111110" report "Invalid read value for IE register:" & str(d) severity failure;


          wb_write(SIRC_BASE+4,irq3_mask); -- Enable IRQ 3 (GPIO_RISE)
          wb_write(GPIO_BASE+8,32ub"0"); -- Disable all GPIO Outputs
          wb_write(GPIO_BASE+4,32ub"1"); -- Enable GPIO input 0
          wb_write(GPIO_BASE+X"18",32ub"1"); -- Enable GPIO Rise Interrupt
          assert sirq_request='0' report "sirq_request is not 0 initially" severity failure;
          wb_read(SIRC_BASE,d);
          assert d=32ub"0" report "IP register is not cleared" severity failure;
          --- Trigger interrupt
          gpio_i(0) <= '1';
          wait until sirq_request = '1';
          wb_read(SIRC_BASE,d);
          assert d=irq3_mask report "IP flag not set correctly" & str(d) severity failure;
          check_sirq_claim(32ux"3");

          -- Clear interrupt source
          wb_write(GPIO_BASE+X"18",32ub"0"); -- Disable GPIO Rise Interrupt
          wb_write(GPIO_BASE+X"1C",32ub"1"); -- Clear IRQ Pending flag
          wait for 2*clk_i_period;
          assert sirq_request = '1' report "sirq_request should still be asserted" severity failure;

          wb_write(SIRC_BASE+8,32ux"3"); -- confirm IRQ
          wait until rising_edge(clk_i);
          assert sirq_request = '0' report "sirq_request not deasserted" severity failure;
          check_sirq_claim(32ub"0");

          print(OUTPUT,"Testing SIRC OK");

        end procedure;


        variable ctl : std_logic_vector(31 downto 0);
        constant Teststr : string := "The quick brown fox jumps over the lazy dog";

   begin
      -- hold reset state for 100 ns.
      wait for 100 ns;
      print(OUTPUT,"UART_0_BASE: " & hstr(std_logic_vector(UART_0_BASE)));
      print(OUTPUT,"FLASH_SPI_BASE: " & hstr(std_logic_vector(FLASH_SPI_BASE)));
      print(OUTPUT,"UART_1_BASE: " & hstr(std_logic_vector(UART_1_BASE)));
      print(OUTPUT,"GPIO_BASE: " & hstr(std_logic_vector(GPIO_BASE)));
      print(OUTPUT,"SIRC_BASE: " & hstr(std_logic_vector(SIRC_BASE)));

      test_spi_loopback;

      if not SKIP_UART_TEST then

        -- UART 0/1 Test
        ctl:=(others=>'0');
        ctl(15 downto 0):=std_logic_vector(to_unsigned(51,16)); -- Divisor 51 for 115200 Baud
        ctl(16):='1';
        wb_write(UART_0_BASE+4,ctl);  -- Initalize UART
        wb_write(UART_1_BASE+4,ctl);  -- Initalize UART

        print(OUTPUT,"Send string: " & Teststr & " to UART0/1");
        -- UART Send Simulation
        for i in 1 to TestStr'length loop
           uart_tx(0,char_to_ascii_byte(TestStr(i)));
           uart_tx(1,char_to_ascii_byte(TestStr(i)));
        end loop;
        uart_tx(0,X"1A"); -- eof
        uart_tx(1,X"1A"); -- eof

        wait until uart0_stop and uart1_stop;



        print(OUTPUT,"UART0 Test captured bytes: " & str(total_count(0)) & " framing errors: " & str(framing_errors(0)));
        assert total_count(0)=TestStr'length+1 and framing_errors(0)=0 severity failure;

         print(OUTPUT,"UART1 Test captured bytes: " & str(total_count(1)) & " framing errors: " & str(framing_errors(1)));
        assert total_count(1)=TestStr'length+1 and framing_errors(1)=0 severity failure;

    else
      print(OUTPUT,"Warning: UART Test skipped");

    end if;

      -- GPIO Test

      print(OUTPUT,"Testing GPIO Output");

      wb_write(GPIO_BASE+8,X"FFFFFFFF"); -- Output Enable
      wait for clk_i_period*2;
      assert gpio_t = X"00000000" report "GPIO Test, Output enable fail: " & str(gpio_t) severity error;

      ctl:=X"00000001";
      for i in 1 to 32 loop
        print(OUTPUT,"Test GPIO with pattern " & str(ctl));
        wb_write(GPIO_BASE+12,ctl);
        wait  until rising_edge(clk_i);
        assert gpio_o = ctl report "GPIO Output test failure" severity error;
        ctl:= ctl(30 downto 0) & '0'; -- shift left
      end loop;
      print(OUTPUT,"OK");

      --SIRC Test
      sirc_test;


      report "Test successfull";
      wait for 10*clk_i_period;

      -- insert stimulus here
      tbSimEnded <= '1';
      wait;
   end process;

END;
