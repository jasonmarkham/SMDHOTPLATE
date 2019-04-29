
uint8_t buzzermode = 0;
uint8_t buzzerstate = 0; //on or off
long BuzzerTimestamp = 0; //for alarms and other beep stuff


void BuzzerAlarm(void)
{
  static uint8_t beepcounter = 0;

  if (buzzermode != 1) //first time call
  {
    BuzzerTimestamp = millis() + 5000;
    buzzermode = 1;
    beepcounter = 0;
    buzzerstate = 0;
  }

  if (beepcounter < 5)
  {
    if ((millis() - BuzzerTimestamp) > 800) //time to switch the beep state
    {
      if (buzzerstate == 0)
      {
        analogWrite(BUZZER_PIN, 128);
        BuzzerTimestamp = millis();
        buzzerstate = 1;
      }
      else
      {
        digitalWrite(BUZZER_PIN, LOW);
        BuzzerTimestamp = millis();
        buzzerstate = 0;
        beepcounter++;
      }
    }
  }
  else buzzermode = 0;
}

void BuzzerShortbeep(void)
{

  if (buzzermode != 2) //first time call
  {
    BuzzerTimestamp = millis()+5000;
    buzzermode = 2;
    buzzerstate = 0;
  }


  if ((millis() - BuzzerTimestamp) > 200)
  {
    
    if (buzzerstate == 0)
      {
        analogWrite(BUZZER_PIN, 128);
        BuzzerTimestamp = millis();
        buzzerstate = 1;
      }
      else
      {
        digitalWrite(BUZZER_PIN, LOW);
        buzzerstate = 0;
        buzzermode = 0;
      }
  }
}

void BuzzerLongbeep(void)
{

  if (buzzermode != 3) //first time call
  {
    BuzzerTimestamp = millis()+5000;
    buzzermode = 3;
    buzzerstate = 0;
  }


  if ((millis() - BuzzerTimestamp) > 1200)
  {
    
    if (buzzerstate == 0)
      {
        analogWrite(BUZZER_PIN, 128);
        BuzzerTimestamp = millis();
        buzzerstate = 1;
      }
      else
      {
        digitalWrite(BUZZER_PIN, LOW);
        buzzerstate = 0;
        buzzermode = 0;
      }
  }
}

//call this function repeatedly
void BuzzerHandler(void)
{
  if (buzzermode > 0)
  {
    if (buzzermode == 1)  BuzzerAlarm();
    else if (buzzermode == 2)  BuzzerShortbeep();
    else if (buzzermode == 3)  BuzzerLongbeep();

  }



}
