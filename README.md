# L9963E_lib

**L9963E_lib** is a low-level communication library designed to interface any microcontroller with the [ST L9963E](https://www.st.com/en/automotive-analog-and-power/l9963e.html) Automotive Multicell Battery Monitoring and Balancing IC. This library facilitates both register-level access and burst commands, streamlining the integration of the L9963E into your battery management systems. It is specifically designed to work in combination with the **L9963T transceiver** and the **L9963E chip**.

## Features

- **Register-Level Access**: Direct interaction with the L9963E's registers for precise control.
- **Burst Command Implementation**: Efficient handling of multiple commands in sequence.
- **Modular Design**: Separation between low-level driver functions and higher-level logic for flexibility.
- **Interface Module**: Provides a template for bridging the library with the hardware abstraction layer (HAL) of your microcontroller. Users must implement their own platform-specific interface functions.

## Getting Started

To utilize L9963E_lib in your project, follow these steps:

### Prerequisites

- **Hardware**:
  - Microcontroller compatible with the L9963E IC
  - L9963T transceiver
- **Software**:
  - C Compiler (e.g., GCC)

### Installation

1. **Clone the Repository**:

   ```bash
   git clone https://github.com/squadracorsepolito/L9963E_lib.git
   ```

2. **Verify Register Files**:

   The `registers.h` file, necessary for interacting with the L9963E, is already included in the repository. No additional generation steps are required.

3. **Integrate into Your Project**:

   - Include the `inc/*.h` headers in your source files.
   - Add the source files `L9963E.c` and `L9963E_DRV.c` to your project build.

## Usage

1. **Initialize the Interface**:

   Set up the communication interface (e.g., SPI) between your microcontroller, the L9963T transceiver, and the L9963E IC.

2. **Implement the Interface Module**:

   The library includes an interface module as a template to bridge its functionality with the HAL of your microcontroller. You must implement the required low-level functions (e.g., SPI transmit/receive, delay functions) specific to your platform. These functions serve as the backbone for communication between the library and the hardware.

3. **Configure the L9963E**:

   Use the provided functions to configure the L9963E according to your application's requirements.

4. **Monitor and Balance Cells**:

   Implement the necessary logic to monitor battery cell parameters and perform balancing operations using the library's functions.

## Contributing

We welcome contributions to enhance L9963E_lib. To contribute:

1. **Fork the Repository**:

   Click the "Fork" button at the top right of the repository page.

2. **Create a Branch**:

   ```bash
   git checkout -b feature/YourFeatureName
   ```

3. **Commit Your Changes**:

   ```bash
   git commit -m 'feat: Add new feature'
   ```

4. **Push to the Branch**:

   ```bash
   git push origin feature/YourFeatureName
   ```

5. **Open a Pull Request**:

   Navigate to your forked repository and click the "New Pull Request" button.

## License

"THE BEER-WARE LICENSE" (Revision 69):
Squadra Corse firmware team wrote this project. As long as you retain this notice
you can do whatever you want with this stuff. If we meet some day, and you
think this stuff is worth it, you can buy us a beer in return.

Authors
- Filippo Rossi <filippo.rossi.sc@gmail.com>
- Federico Carbone <federico.carbone.sc@gmail.com>

## Acknowledgments

Special thanks to the members of [Squadra Corse PoliTO](https://github.com/squadracorsepolito) for developing and maintaining this library.

For more information, visit our [GitHub page](https://github.com/squadracorsepolito/L9963E_lib).

