#include "stm32f4xx_hal.h"
#include "main.h"
#include "bq25703_regmap.h"
extern I2C_HandleTypeDef hi2c1;
#define TIMEOUT 100
#define MAXTRIES 10
#if 1
  void die(uint16_t ms)
  {
    LED_GPIO_Port->ODR ^= LED_Pin;
    HAL_Delay(ms);
  }
  void led_on()
  {
    LED_GPIO_Port->BSRR = LED_Pin;
  }
  void led_off()
  {
    LED_GPIO_Port->BSRR = (uint32_t)LED_Pin << 16U;
  }
#endif
  
uint32_t bq_write(uint8_t *pbuf, uint8_t size, uint8_t addr)
{
  HAL_StatusTypeDef status = HAL_OK;
  status = HAL_I2C_Mem_Write(&hi2c1, BQ25703_I2C_ADDR, (uint16_t)addr, 
                              I2C_MEMADD_SIZE_8BIT, pbuf, size, TIMEOUT);
  #if 0
    if(status != HAL_OK)
    {
      // handler
    }
    while(HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
    {}
    while(HAL_I2C_IsDeviceReady(&hi2c1, BQ25703_I2C_ADDR, MAXTRIES, TIMEOUT)
          == HAL_TIMEOUT)
    {}
    while(HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
    {}
  #endif
    return status;
}

uint32_t bq_read(uint8_t *pbuf, uint8_t size, uint8_t addr)
{
  HAL_StatusTypeDef status = HAL_OK;
  status = HAL_I2C_Mem_Read(&hi2c1, BQ25703_I2C_ADDR, (uint16_t)addr, I2C_MEMADD_SIZE_8BIT, pbuf, size, TIMEOUT);
  return status;
}
uint32_t bq_write_bytes_2_addrs(uint8_t *pbuf, uint8_t size, uint8_t *addr)
{
  HAL_StatusTypeDef status = HAL_OK;
  for(uint8_t i = 0; i < size; i++)
    status = HAL_I2C_Mem_Read(&hi2c1, BQ25703_I2C_ADDR, (uint16_t)(addr[i]), I2C_MEMADD_SIZE_8BIT, pbuf+i, 1, TIMEOUT);
  return status; 
}
/* 
  cfg[0]    lower addr
  cfg[1]    lower reg value
  cfg[2]    higher reg value
*/
uint32_t bq_write16(uint8_t cfg[3])
{
  HAL_StatusTypeDef status = HAL_OK;
  status = HAL_I2C_Mem_Write(&hi2c1, BQ25703_I2C_ADDR, (uint16_t)(cfg[0]), I2C_MEMADD_SIZE_8BIT, cfg+1, 1, TIMEOUT);
  status = HAL_I2C_Mem_Write(&hi2c1, BQ25703_I2C_ADDR, (uint16_t)(cfg[0]+1), I2C_MEMADD_SIZE_8BIT, cfg+2, 1, TIMEOUT);
  return status;

}
/* Sense resistor configurations and macros */
#define DEFAULT_SENSE_RESISTOR 10
#define INPUT_RESISTOR_RATIO \
	((CONFIG_CHARGER_SENSE_RESISTOR_AC) / DEFAULT_SENSE_RESISTOR)
#define REG_TO_INPUT_CURRENT(REG) ((REG + 1) * 50 / INPUT_RESISTOR_RATIO)
#define INPUT_CURRENT_TO_REG(CUR) (((CUR) * INPUT_RESISTOR_RATIO / 50) - 1)
#define CHARGING_RESISTOR_RATIO \
	((CONFIG_CHARGER_SENSE_RESISTOR) / DEFAULT_SENSE_RESISTOR)
#define REG_TO_CHARGING_CURRENT(REG) ((REG) / CHARGING_RESISTOR_RATIO)
#define CHARGING_CURRENT_TO_REG(CUR) ((CUR) * CHARGING_RESISTOR_RATIO)
/* Console output macros */
//#define CPRINTF(format, args...) cprintf(CC_CHARGER, format, ## args)
/* Charger parameters */
//static const struct charger_info bq25703_charger_info = 
//{
//	.name         = "bq25703",
//	.voltage_max  = 19200,
//	.voltage_min  = 1024,
//	.voltage_step = 16,
//	.current_max  = 8128 / CHARGING_RESISTOR_RATIO,
//	.current_min  = 64 / CHARGING_RESISTOR_RATIO,
//	.current_step = 64 / CHARGING_RESISTOR_RATIO,
//	.input_current_max  = 6400 / INPUT_RESISTOR_RATIO,
//	.input_current_min  = 50 / INPUT_RESISTOR_RATIO,
//	.input_current_step = 50 / INPUT_RESISTOR_RATIO,
//};
static inline uint8_t raw_read8(uint8_t offset, uint8_t *value)
{
	return bq_read(value, 1, offset);
}
static inline uint8_t raw_write8(uint8_t offset, uint8_t *value)
{
	return bq_write(value, 1, offset);
}
static inline uint8_t raw_read16(uint8_t offset, uint8_t *value)
{
	return bq_read(value, 2, offset);
}
static inline uint8_t raw_write16(uint8_t offset, uint8_t *value)
{
	return bq_write(value, 2, offset);
}
#if 0
#if 1 || CONFIG_CHARGE_RAMP_HW
static uint8_t bq25703_get_low_power_mode(uint8_t *mode)
{
	uint8_t rv;
	uint8_t reg;
	rv = raw_read16(BQ25703_REG_CHARGE_OPTION_0, &reg);
	if (rv)
		return rv;
	*mode = !!(reg & BQ25703_CHARGE_OPTION_0_LOW_POWER_MODE);
	return EC_SUCCESS;
}
static uint8_t bq25703_set_low_power_mode(uint8_t enable)
{
	uint8_t rv;
	uint8_t reg;
	rv = raw_read16(BQ25703_REG_CHARGE_OPTION_0, &reg);
	if (rv)
		return rv;
	if (enable)
		reg |= BQ25703_CHARGE_OPTION_0_LOW_POWER_MODE;
	else
		reg &= ~BQ25703_CHARGE_OPTION_0_LOW_POWER_MODE;
	rv = raw_write16(BQ25703_REG_CHARGE_OPTION_0, reg);
	if (rv)
		return rv;
	return EC_SUCCESS;
}
#endif
/* Charger interfaces */
//const struct charger_info *charger_get_info(void)
//{
//	return &bq25703_charger_info;
//}
uint8_t charger_post_init(void)
{
	/*
	 * Note: bq25703 power on reset state is:
	 *	watch dog timer     = 175 sec
	 *	input current limit = ~1/2 maximum setting
	 *	charging voltage    = 0 mV
	 *	charging current    = 0 mA
	 *	discharge on AC     = disabled
	 */
	/* Set charger input current limit */
	return charger_set_input_current(CONFIG_CHARGER_INPUT_CURRENT);
}
uint8_t charger_get_status(uint8_t *status)
{
	uint8_t rv;
	uint8_t option;
	rv = charger_get_option(&option);
	if (rv)
		return rv;
	/* Default status */
	*status = CHARGER_LEVEL_2;
	if (option & BQ25703_CHARGE_OPTION_0_CHRG_INHIBIT)
		*status |= CHARGER_CHARGE_INHIBITED;
	return EC_SUCCESS;
}
uint8_t charger_set_mode(uint8_t mode)
{
	uint8_t rv;
	uint8_t option;
	rv = charger_get_option(&option);
	if (rv)
		return rv;
	if (mode & CHARGER_CHARGE_INHIBITED)
		option |= BQ25703_CHARGE_OPTION_0_CHRG_INHIBIT;
	else
		option &= ~BQ25703_CHARGE_OPTION_0_CHRG_INHIBIT;
	return charger_set_option(option);
}
uint8_t charger_enable_otg_power(uint8_t enabled)
{
	/* This is controlled with the EN_OTG pin. Support not added yet. */
	return EC_ERROR_UNIMPLEMENTED;
}
uint8_t charger_set_otg_current_voltage(uint8_t output_current, uint8_t output_voltage)
{
	/* Add when needed. */
	return EC_ERROR_UNIMPLEMENTED;
}
uint8_t charger_is_sourcing_otg_power(uint8_t port)
{
	/* Add when needed. */
	return EC_ERROR_UNIMPLEMENTED;
}
uint8_t charger_get_current(uint8_t *current)
{
	uint8_t rv, reg;
	rv = raw_read16(BQ25703_REG_CHARGE_CURRENT, &reg);
	if (!rv)
		*current = REG_TO_CHARGING_CURRENT(reg);
	return rv;
}
uint8_t charger_set_current(uint8_t current)
{
	return raw_write16(BQ25703_REG_CHARGE_CURRENT,
		CHARGING_CURRENT_TO_REG(current));
}
/* Get/set charge voltage limit in mV */
uint8_t charger_get_voltage(uint8_t *voltage)
{
	return raw_read16(BQ25703_REG_MAX_CHARGE_VOLTAGE, voltage);
}
uint8_t charger_set_voltage(uint8_t voltage)
{
	return raw_write16(BQ25703_REG_MAX_CHARGE_VOLTAGE, voltage);
}
/* Discharge battery when on AC power. */
uint8_t charger_discharge_on_ac(uint8_t enable)
{
	uint8_t rv, option;
	rv = charger_get_option(&option);
	if (rv)
		return rv;
	if (enable)
		option |= BQ25703_CHARGE_OPTION_0_EN_LEARN;
	else
		option &= ~BQ25703_CHARGE_OPTION_0_EN_LEARN;
	return charger_set_option(option);
}
uint8_t charger_set_input_current(uint8_t input_current)
{
	return raw_write8(BQ25703_REG_IIN_HOST,
		INPUT_CURRENT_TO_REG(input_current));
}
uint8_t charger_get_input_current(uint8_t *input_current)
{
	uint8_t rv, reg;
	/*
	 * IIN_DPM register reflects the actual input current limit programmed
	 * in the register, either from host or from ICO. After ICO, the
	 * current limit used by DPM regulation may differ from the IIN_HOST
	 * register settings.
	 */
	rv = raw_read8(BQ25703_REG_IIN_DPM, &reg);
	if (!rv)
		*input_current = REG_TO_INPUT_CURRENT(reg);
	return rv;
}
uint8_t charger_manufacturer_id(uint8_t *id)
{
	return raw_read8(BQ25703_REG_MANUFACTURER_ID, id);
}
uint8_t charger_device_id(uint8_t *id)
{
	return raw_read8(BQ25703_REG_DEVICE_ADDRESS, id);
}
uint8_t charger_get_option(uint8_t *option)
{
	/* There are 4 option registers, but we only need the first for now. */
	return raw_read16(BQ25703_REG_CHARGE_OPTION_0, option);
}
uint8_t charger_set_option(uint8_t option)
{
	/* There are 4 option registers, but we only need the first for now. */
	return raw_write16(BQ25703_REG_CHARGE_OPTION_0, option);
}
#ifdef CONFIG_CHARGE_RAMP_HW
static void bq25703_chg_ramp_handle(void)
{
	uint8_t ramp_curr;
	/*
	 * Once the charge ramp is stable write back the stable ramp
	 * current to input current register.
	 */
	if (chg_ramp_is_stable()) {
		ramp_curr = chg_ramp_get_current_limit();
		if (ramp_curr && !charger_set_input_current(ramp_curr))
			CPRINTF("stable ramp current=%d\n", ramp_curr);
	}
}
DECLARE_DEFERRED(bq25703_chg_ramp_handle);
uint8_t charger_set_hw_ramp(uint8_t enable)
{
	uint8_t option3_reg, option2_reg, rv;
	rv = raw_read16(BQ25703_REG_CHARGE_OPTION_3, &option3_reg);
	if (rv)
		return rv;
	rv = raw_read16(BQ25703_REG_CHARGE_OPTION_2, &option2_reg);
	if (rv)
		return rv;
	if (enable) {
		/* Set InputVoltage register to BC1.2 minimum ramp voltage */
		rv = raw_write16(BQ25703_REG_INPUT_VOLTAGE,
			BQ25703_BC12_MIN_VOLTAGE_MV);
		if (rv)
			return rv;
		/*  Enable ICO algorithm */
		option3_reg |= BQ25703_CHARGE_OPTION_3_EN_ICO_MODE;
		/* 0b: Input current limit is set by BQ25703_REG_IIN_HOST */
		option2_reg &= ~BQ25703_CHARGE_OPTION_2_EN_EXTILIM;
		/* Charge ramp may take up to 2s to settle down */
		hook_call_deferred(&bq25703_chg_ramp_handle_data, (4 * SECOND));
	} else {
		/*  Disable ICO algorithm */
		option3_reg &= ~BQ25703_CHARGE_OPTION_3_EN_ICO_MODE;
		/*
		 * 1b: Input current limit is set by the lower value of
		 * ILIM_HIZ pin and BQ25703_REG_IIN_HOST
		 */
		option2_reg |= BQ25703_CHARGE_OPTION_2_EN_EXTILIM;
	}
	rv = raw_write16(BQ25703_REG_CHARGE_OPTION_2, option2_reg);
	if (rv)
		return rv;
	return raw_write16(BQ25703_REG_CHARGE_OPTION_3, option3_reg);
}
uint8_t chg_ramp_is_stable(void)
{
	uint8_t reg;
	if (raw_read16(BQ25703_REG_CHARGER_STATUS, &reg))
		return 0;
	return reg & BQ25703_CHARGE_STATUS_ICO_DONE;
}
uint8_t chg_ramp_get_current_limit(void)
{
	uint8_t reg;
	uint8_t mode;
	uint8_t tries_left = 8;
	/* Save current mode to restore same state after ADC read */
	if (bq25703_get_low_power_mode(&mode))
		goto error;
	/* Exit low power mode so ADC conversion takes typical time */
	if (bq25703_set_low_power_mode(0))
		goto error;
	/* Turn on the ADC for one reading */
	reg = BQ25703_ADC_OPTION_ADC_START | BQ25703_ADC_OPTION_EN_ADC_IIN;
	if (raw_write16(BQ25703_REG_ADC_OPTION, reg))
		goto error;
	/*
	 * Wait until the ADC operation completes. The spec says typical
	 * conversion time is 10 msec. If low power mode isn't exited first,
	 * then the conversion time jumps to ~60 msec.
	 */
	do {
		msleep(2);
		raw_read16(BQ25703_REG_ADC_OPTION, &reg);
	} while (--tries_left && (reg & BQ25703_ADC_OPTION_ADC_START));
	/* ADC reading attempt complete, go back to low power mode */
	if (bq25703_set_low_power_mode(mode))
		goto error;
	/* Could not complete read */
	if (reg & BQ25703_ADC_OPTION_ADC_START)
		goto error;
	/* Read ADC value */
	if (raw_read8(BQ25703_REG_ADC_IIN, &reg))
		goto error;
	/* LSB => 50mA */
	return reg * BQ25703_ADC_IIN_STEP_MA;
error:
	CPRINTF("Could not read input current limit ADC!\n");
	return 0;
}
#endif /* CONFIG_CHARGE_RAMP_HW */
#endif