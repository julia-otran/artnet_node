# ESP12-E + ATtiny DMX512 ArtNet

## Under Construction

### Basic Components

- (1x) ESP12-E
- (4x) ATtiny1614
- (8x) MAX485 or similar RS-232 <-> RS-485 converter

### Multiple universes

- 4 input universes
- 4 output universes

### Basic Diagram

Connect the ESP SPI interface to the ATtiny SPI.

- ESP sends and receives dmx data via SPI
- ATtiny "converts" SPI data into Serial data, and vice-versa.
