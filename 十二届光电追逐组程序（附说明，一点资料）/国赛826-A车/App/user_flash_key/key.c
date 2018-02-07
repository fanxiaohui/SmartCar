#include "common.h"
#include "include.h"

/***********************************************************************************************************************************/

/***********************************************************************************************************************************/

uint8 once_check_key()
{
      if(UP_KEY==0)
      {
            KEY_DELAY;
            if(UP_KEY==0)
            {
                  while(!UP_KEY);
                  return UP;
            }
      }
      else if(DOWN_KEY==0)
      {
            KEY_DELAY;
            if(DOWN_KEY==0)
            {
                  while(!DOWN_KEY);
                  return DOWN;
            }
      }
      else if(LEFT_KEY==0)
      {
            KEY_DELAY;
            if(LEFT_KEY==0)
            {
                  while(!LEFT_KEY);
                  return LEFT;
            }
      }
      else if(RIGHT_KEY==0)
      {
            KEY_DELAY;
            if(RIGHT_KEY==0)
            {
                  while(!RIGHT_KEY);
                  return RIGHT;
            }
      }
      else if(OK_KEY==0)
      {
            KEY_DELAY;
            if(OK_KEY==0)
            {
                  while(!OK_KEY);
                  return OK;
            }
      }
      return 0;
}

/***********************************************************************************************************************************/


