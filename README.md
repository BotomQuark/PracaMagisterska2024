# Praca Magisterska
- Autor: Krzysztof Borowski,
- Promotor: dr inż. Piotr Kaczmarek,
- Tytuł pracy: Adaptacyjny regulator prędkości obrotowej silnika prądu stałego,
- Angielski tytuł pracy: Adaptive DC motor speed controller.


# Skrypty matlab
| Nazwa pliku | Opis |
|-------------|------|
|Open_system.m| Skrypt matlab, który pozwala na zebranie charakterystyki czasowej ukladu otwartego. | 
|Closed_system.m| Skrypt matlab, który pozwala na sterowanie obiektu za pomocą regulatora PI w układzie zamkniętym.|
|EwRLS.m| Skrypt matlab implementujący identyfikację obiektu metodą rekursywnych najmniejszych kwadratów z zapominaniem wykładniczym w układzie otwartym.|
|EwRLS_ZPP.m| Skrypt matlab implementujący adaptacyjne sterowanie metodą pozycjonowania zer i biegunów za pomocą identyfikacji obiektu. |
|Mras_mit.m| Skrypt matlab implementujący adaptacyjne sterowanie z obiektem odniesienia za pomocą reguły MIT. |
|Mras_mit_norm.m| Skrypt matlab implementujący adaptacyjne sterowanie z obiektem odniesienia za pomocą znormalizowanej reguły MIT. |
|Mras_mit_norm_sine.m| Skrypt matlab implementujący adaptacyjne sterowanie z obiektem odniesienia za pomocą znormalizowanej reguły MIT przy sinusoidalnym sygnale referencyjnym. |


# Projekty CubeIDE
W folderach `U5_RLS_ZPP` oraz `U5_MRAS` znajdują się projekty stworzone za pomocą programu CubeMX na płytki NUCLEO-U575ZI-Q.
Projekty różnią się częstotliwością z którą wysyłane są dane dotyczące prędkości obrotowej silnika DC poprzez UART.

Projekt `U5_RLS_ZPP` zostal napisany w taki sposób dane wysyłane były z częstotliwością 7 milisekund i należy go używać wraz z skryptami zaczynającymi swoją nazwę od `EwRLS...`.

Projekt `U5_MRAS` został napisany w taki sposób by dane wysyłane były z częstotliwością 2 milisekund i należy go uzywać wraz z skryptami zaczynającymi swoją nazwę od  `Mras...`.
