#include <stdio.h>

void search_apartment_position(unsigned int entrances, unsigned int floors, unsigned int apartment_number);

int main( void ) {
   unsigned int entrances = 0;
   unsigned int floors = 0;
   unsigned int apartment_number = 0;

   printf("%s", "Введите количество подъездов, этажей и номер искомой квартиры: ");
   if (scanf("%d %d %d", &entrances, &floors, &apartment_number) != 3) {
      puts("Введены некорректные данные.");
      return -1;
   }
   
   if (entrances == 0 || floors < 2) {
      puts("Такого многоквартирного дома быть не может, пожалуйста введите данные корректно.");
      return -1;
   }
   
   search_apartment_position(entrances, floors, apartment_number);
}

void search_apartment_position( unsigned int entrances, unsigned int floors, unsigned int apartment_number ) {
   int apartaments_per_entrance = floors * 4;

   unsigned int entrance = ( apartment_number - 1 ) / apartaments_per_entrance + 1;
   unsigned int floor = ( ( apartment_number - 1 ) % apartaments_per_entrance ) / 4 + 1;
   unsigned int position = ( apartment_number - 1 ) % 4 + 1;
   unsigned int apartment_existence = (apartment_number > 0) & (entrance > 0) & (entrance <= entrances);

   printf(apartment_existence ? "Подъезд: %d, Этаж: %d, Позиция: %d\n" : "Квартиры с таким номером нет в доме\n", entrance, floor, position);
}
