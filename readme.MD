# bonfire-soc-io
IO block of bonfire-soc



## Simple_IRC Module
The Simple IRC Module contains a simple Interrupt Controller

### Configuration:
Parameter     | Type        | Description                | Default
--------------|-------------|----------------------------|---------------
NUM_IRQ       | natural     | 1..32: number of IRQ Lines | 8

### Registers:

Name              | Address     | Description  | Bits
------------------|-------------|-----------------------|---------------
IP                | 0           | IRQ Pending (details see below)) |  n(1..NUM_IRQ-1) : Pending irq_in (n)
IE                | 4           | Interupt Enable 1: Enabled, 0: Disabled |  n(1..NUM_IRQ-1) : Enable irq_in (n)
ICLAIM            | 8         | IRQ Claim/Complete register | 5:0: Number of highest prio IRQ


### Operation

The SIRC is a very simplified interrupt controller which borrows some concepts from the RISC-V PLIC.

All IRQs have a fixed priority, irq_in(1) has the lowest, irq_n(NUM_IRQ) the highest.
All IRQs have also a number (used in the claim/complete process), irq_in(1) has number 1 and irq_n has NUM_IRQ. 
Interrupt number 0 does not exist, therefore also bit 0 if IP and IE are hardwired to 0

To enable an Interrupt(n) the coressponding bit IE(n) has to be set to 1

The corresponding bit IP(n) is set when IE(n) and irq_in(n) is 1 at the rising edge of clk_i. IP(n) stays high until the Interrupt is claimed

#### ICLAIM Register
On Read the ICLAIM register will either return 0 when there is no pending interrupt, or it contains the interrupt number of highest pending irq. Reading ICLAIM is stateless, so it will always return the highest pending irq at the time of read. 

Writing an Interrupt number to the ICLAIN Register will clear the corresponding interrupt.


### irq_req_o signal
The signal irq_req_o will stay '1' as long as the at least one pending interrupt. So clearing an interrupt will keep the signal asserted until there are no pending interrupts left. 







