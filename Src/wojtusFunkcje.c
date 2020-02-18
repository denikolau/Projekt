#include <wojtusFunkcje.h>

/**
 *  @brief Ta funkcja obslugujaca PID przyjmuje 2 argumenty i zadaje wartosc wypelnienia pwm
 *  @param varZ Pierwszy argument typu int bedacy wartoscia zadana
 *  @param varO Drugi argument typu int bedacy wartoscia odczytana z czujnika
 *  @param PID Trzeci argument jest wskaznikiem na instancje regulatora
 *  @retval brak
 */
void obliczPID(int varZ, int varO, arm_pid_instance_f32 * PID)
{
	int wypelnienie;
	wypelnienie = arm_pid_f32(PID, varZ - varO); //obliczanie wypelnienia dzieki funkcji z biblioteki cmsis
	/* Ogranicznik */
	if (wypelnienie < 0) wypelnienie = 0;
	else if (wypelnienie > 1000) wypelnienie = 1000;
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, wypelnienie); //Zadaje wypelnienie PWM
}




/**
 *  @brief Ta funkcja odczytuje natezenie swiatla z czujnika BH1750, nie przyjmuje zadnych argumentow

 *  @retval Wartosc natezenia wyrazona w LUX
 */
int odczytBH1750()
{
	float odczyt_float;
	int odczyt_int;
	BH1750_ReadLight(&odczyt_float); //ODCZYT WARTOSCI DO PRZEKAZANEJ ZMIENNEJ
	odczyt_int = odczyt_float;
	return odczyt_int;
}


/**
 *  @brief Ta funkcja ma na celu odswiezac wyswietlacz LCD poprzez usuniecie wszystkich
 *  znakow i ponowne zapisanie ich z czestotliowscia uzalezniona od zegara przez ktory bedzie wywolywana
 *
 *  @param varZ Pierwszy argument typu int bedacy wartoscia zadana z zakresu 0-99999999
 *  @param varO Drugi argument typu int bedacy wartoscia odczytana z czujnika z zakresu 0-99999999
 *
 *  @retval brak
 */
void obslugaLCD(int varZ, int varO)
{
		  char bufor_wiadomosci[16];
		  lcd_clear();
		  lcd_put_cur(0, 0); // od tego miejsca zaczynamy wprowadzac znaki
		  if(varZ > 9999999) // zabezpieczenie
		  {
			  lcd_send_string("Za duza liczba");
		  }
		  else if(varZ<-9999999)
		  {
			  lcd_send_string("Za mala liczba");
		  }
		  else
		  {
		  sprintf(&bufor_wiadomosci, "Zadana: %d", varZ);
		  lcd_send_string(bufor_wiadomosci); // wysylanie wiadomosci
		  lcd_put_cur(1, 0); // zmiana miejsca wpisywania tekstu na druga linie
		  sprintf(&bufor_wiadomosci, "Odczyt: %d", varO);
		  lcd_send_string(bufor_wiadomosci); // wysylanie wiadomosci
		  }
}
