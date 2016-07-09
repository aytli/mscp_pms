#include "main.h"


void main()
{

   setup_adc_ports(sAN0);
   setup_timer_4(T4_DISABLED,0,1);
   setup_comparator(NC_NC_NC_NC);// This device COMP currently not supported by the PICWizard


}
