/*
 * N2 (steering zone). CAN part copied from lab3_stm32/can_bringup_test, servo
 * and blinker pins from lab2_stm32.
 *
 *   SPI1: SCK PA5, MISO PA6, MOSI PA7 (both MCP2515 modules)
 *   CAN A: CS PB6 [D10], INT PA10 [D2]    CAN B: CS PC7 [D9], INT PB5 [D4]
 *   Servo:   PB10 [D6] TIM2_CH3, 50 Hz           (test point row 9  SRV)
 *   Blinker: FL PA15 [CN7-17], FR PA1 [A1]       (rows 10, 11)
 *   Test button: B1 (PC13), the board's sw0
 *
 * Test points (Lab 3 test-point table, N2 column):
 *   row 3 CAN_RX_A  PC8  [CN10-2]     row 4 CAN_RX_B  PC9  [CN10-1]
 *   row 5 CAN_TX_A  PC10 [CN7-1]      row 6 CAN_TX_B  PC11 [CN7-2]
 *   row 7 DEG_A LED PA8  [D7]         row 8 DEG_B LED PA9  [D8]
 *   row 12 RELAY_TX PC12 [CN7-3]      row 13 ERR      PC0  [A5]
 *
 * Board defaults on those pins are turned off: LD2 and the TIM2 PWM LED are on
 * PA5 = SCK, USART1 TX is on PB6, I2C1 is on PB8/PB9, I2C3 on PA8/PC9.
 */
#include <zephyr/dt-bindings/pwm/pwm.h>

/ {
	chosen {
		zephyr,canbus = &can_a;
	};

	aliases {
		/delete-property/ led0;
		/delete-property/ pwm-led0;
		can-a = &can_a;
		can-b = &can_b;
	};

	zephyr,user {
		pwms = <&servo_pwm 3 PWM_MSEC(20) PWM_POLARITY_NORMAL>;
		/* front blinkers: 0 = FL (left), 1 = FR (right) */
		blink-gpios = <&gpioa 15 GPIO_ACTIVE_HIGH>, <&gpioa 1 GPIO_ACTIVE_HIGH>;
		canrxa-gpios = <&gpioc 8 GPIO_ACTIVE_HIGH>;
		canrxb-gpios = <&gpioc 9 GPIO_ACTIVE_HIGH>;
		cantxa-gpios = <&gpioc 10 GPIO_ACTIVE_HIGH>;
		cantxb-gpios = <&gpioc 11 GPIO_ACTIVE_HIGH>;
		dega-gpios = <&gpioa 8 GPIO_ACTIVE_HIGH>;
		degb-gpios = <&gpioa 9 GPIO_ACTIVE_HIGH>;
		relaytx-gpios = <&gpioc 12 GPIO_ACTIVE_HIGH>;
		err-gpios = <&gpioc 0 GPIO_ACTIVE_HIGH>;
	};
};

&{/pwmleds} { status = "disabled"; };
&usart1 { status = "disabled"; };
&i2c1 { status = "disabled"; };
&i2c3 { status = "disabled"; };

/* R4.5: independent watchdog */
&iwdg { status = "okay"; };

/* Servo PWM: TIM2 channel 3 on PB10 (from lab2_stm32). */
&timers2 {
	status = "okay";
	st,prescaler = <0>;
	servo_pwm: pwm {
		status = "okay";
		pinctrl-0 = <&tim2_ch3_pb10>;
		pinctrl-names = "default";
	};
};

&spi1 {
	status = "okay";
	pinctrl-0 = <&spi1_sck_pa5 &spi1_miso_pa6 &spi1_mosi_pa7>;
	pinctrl-names = "default";
	cs-gpios = <&gpiob 6 (GPIO_ACTIVE_LOW | GPIO_PULL_UP)>,
		   <&gpioc 7 (GPIO_ACTIVE_LOW | GPIO_PULL_UP)>;

	can_a: mcp2515@0 {
		compatible = "microchip,mcp2515";
		reg = <0>;
		spi-max-frequency = <10000000>;
		int-gpios = <&gpioa 10 GPIO_ACTIVE_LOW>;
		osc-freq = <16000000>;	/* must match the crystal on the module */
		bitrate = <500000>;
		status = "okay";
	};

	can_b: mcp2515@1 {
		compatible = "microchip,mcp2515";
		reg = <1>;
		spi-max-frequency = <10000000>;
		int-gpios = <&gpiob 5 GPIO_ACTIVE_LOW>;
		osc-freq = <16000000>;	/* must match the crystal on the module */
		bitrate = <500000>;
		status = "okay";
	};
};
