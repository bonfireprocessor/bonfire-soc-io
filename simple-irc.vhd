----------------------------------------------------------------------------------
-- 
-- 
-- Module Name:    sysio - Behavioral


-- The Bonfire Processor Project, (c) 2022 Thomas Hornschuh

-- Bonfire CPU Toplevel module with Block RAM und WISHBONE interfaces
-- Includes Instruction Cache

-- License: See LICENSE or LICENSE.txt File in git project root.

-- simple_irc:  A simple Interrupt controller

-- Registers:



----------------------------------------------------------------------------------


library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;

entity simple_irc is
generic (
   NUM_IRQ : natural := 8  
);
Port (
   clk_i: in std_logic;
   rst_i: in std_logic;
   
   irq_in : in  std_logic_vector(NUM_IRQ downto 1);
  
   irq_req_o: out std_logic ; 
   
   -- Slave Interface 
   
   wbs_cyc_i : in std_logic ;
   wbs_stb_i : in std_logic ;
   wbs_we_i : in std_logic ;
  
   wbs_ack_o : out std_logic ;
   wbs_adr_i : in std_logic_vector(3 downto 2);
   
   wbs_dat_o : out std_logic_vector(31 downto 0);
   wbs_dat_i : in std_logic_vector(31 downto 0)
);   
end simple_irc;

architecture Behavioral of simple_irc is



subtype  t_irq_bits is std_logic_vector (NUM_IRQ downto 1);
subtype  t_irq_num is natural range 0 to NUM_IRQ;

signal ie_reg : t_irq_bits := (others=>'0');
signal ip_reg : t_irq_bits := (others=>'0');
signal iclaim : t_irq_num;

subtype t_dbus is std_logic_vector(wbs_dat_o'high downto wbs_dat_o'low);

function fill_bits(v: std_logic_vector) return t_dbus is
   variable r : t_dbus;
   begin
     r(v'range):=v;
     r(r'high downto v'length) := (others=>'0');
     return r;
end;

begin

 wbs_ack_o <=  wbs_cyc_i and wbs_stb_i;
 

-- IRQ req and irq number encoder 
process(ip_reg) 
variable tmp : std_logic;
variable max : t_irq_num;
begin

   tmp := '0';
   max := 0;
   for i in  1 to NUM_IRQ loop
      tmp := tmp or ip_reg(i);
      if ip_reg(i)='1' then
         max := i;
      end if;    
   end loop;
   irq_req_o <= tmp;
   iclaim <= max; 

end process;

-- Register read

process(all)  --wbs_cyc_i,wbs_stb_i,wbs_adr_i, wbs_we_i, ie_reg,ip_reg,iclaim)
variable temp_vector : std_logic_vector(NUM_IRQ downto 0);   
begin
   if wbs_cyc_i='1' and wbs_stb_i = '1' and wbs_we_i = '0' then
      case wbs_adr_i is
        when "00" => 
           temp_vector := (ip_reg&'0'); -- pad bit 0
           wbs_dat_o <= fill_bits(temp_vector);
        when "01" =>
           temp_vector := ie_reg&'0';  -- pad bit 0
           wbs_dat_o <= fill_bits(temp_vector); 
        when "10" =>
           wbs_dat_o <= std_logic_vector(to_unsigned(iclaim,wbs_dat_o'length));  
        when others =>
          wbs_dat_o <= (others => 'X');
      end case;
  end if;  
end process;  



 process(clk_i) 
 variable int_num : t_irq_num;
 begin  
   if rising_edge(clk_i) then
     if rst_i='1' then
       ie_reg <=  (others=>'0');
       ip_reg <=  (others=>'0');

     elsif wbs_stb_i='1' and wbs_cyc_i = '1' and wbs_we_i='1' then
        -- Write Registers
        case wbs_adr_i is
        
          when "01" =>
             ie_reg <= wbs_dat_i(ie_reg'range);
         
          when "10" =>
             -- Interrupt completion
             if unsigned(wbs_dat_i(5 downto 0))<= NUM_IRQ then 
                int_num:= to_integer(unsigned(wbs_dat_i(5 downto 0)));
                ip_reg(int_num) <= '0';
             end if;    
                 
          when others =>
             -- nothing 
                    
        end case;
            
      else  
         --- Interrupt sensing, ip_reg is "sticky"
         for i in 1 to NUM_IRQ loop 
            ip_reg(i) <= (ie_reg(i) and irq_in(i)) or ip_reg(i); 
         end loop;    
     end if;  
 
   end if;
 
 end process;


end Behavioral;
