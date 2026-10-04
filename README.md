# Triki Autopilot

Autopilot eksperymentalny dla macOS: analizuje lokalnie widoczne okno QuickTime Player i wysyła syntetyczne próbki IMU przez CoreBluetooth. Wymaga macOS, Bluetooth LE oraz obrazu gry z iPhone'a w widocznym oknie QuickTime.

## Instalacja

W Terminalu przejdź do sklonowanego repozytorium:

```bash
cd triki
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install pyobjc-framework-Cocoa pyobjc-framework-CoreFoundation pyobjc-framework-CoreBluetooth pyobjc-framework-Quartz opencv-python mss numpy
```

## Uruchomienie

1. Podłącz iPhone'a do Maca i otwórz QuickTime Player.
2. Wybierz **Plik → Nowe nagranie filmowe**.
3. Z menu obok przycisku nagrywania wybierz ekran iPhone'a. Zostaw okno z podglądem widoczne.
4. W Terminalu, w katalogu repozytorium, aktywuj środowisko i uruchom bota:

```bash
source .venv/bin/activate
python bot.py
```

Bot automatycznie wyszukuje widoczne okno QuickTime Player; nie trzeba podawać współrzędnych ekranu. Opcjonalnie można wskazać wycinek MSS, podając `--region LEWO GÓRA SZEROKOŚĆ WYSOKOŚĆ`. Opóźnienie klonowania obrazu można dostroić parametrem `--latency-ms`, a dodatkowy czas reakcji statku parametrem `--reaction-ms` (wartości domyślne: odpowiednio 120 ms i 180 ms).

W macOS zezwól używanej aplikacji Terminal na dostęp do **Bluetooth** oraz **Nagrywania ekranu** w **Ustawieniach systemowych → Prywatność i ochrona**. Klawisz `q` lub `Esc` w oknie podglądu kończy działanie.

## Ograniczenia

Obraz jest przetwarzany lokalnie. CoreBluetooth na macOS nie pozwala ustawić statycznego adresu MAC ani wiernie odtworzyć wszystkich pól reklamy BLE. Dokumentacja TrikiEmu wskazuje, że Żappka może wymagać MAC fizycznego kapsla, dlatego połączenie z Żappką nie jest gwarantowane. Skrypt nie zastępuje sprzętowego ESP32, jeśli wymagane jest wierne odtworzenie adresu kapsla.