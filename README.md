# ESP12F (ESP8266) + ATtiny DMX512 ArtNet Interface

### Basic Components

- (1x) ESP12-E
- (1x) SN74AHC138
- (4x) ATtiny1614
- (4x) 6N137
- (4x) MAX487 or similar RS-232 <-> RS-485 converter

### Multiple universes

- 4 synchronized output universes

### Electric Diagram

The electric diagram was built with EasyEDA. Just import the JSON file.

There are 3 Sheets into EasyEDA project.

- The J14 from Sheet1 should be connected to J5 on Sheet2.
- Each one of J10, J11, J12 and J13 should be connected to J2 on Sheet3 (You will need 4 units of Sheet3 Output Isolation module to get the 4 universes and connect each one to the outputs of Sheet2).
