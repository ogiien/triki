# Triki Windows Autopilot

Windows-only bot: Bumble wystawia BLE peripheral bez ESP32, a OpenCV analizuje obraz gry i automatycznie steruje próbkami IMU. Protokół jest oparty na `TrikiEmu/firmware/README.md`.

## Wymagania

- Windows 10/11 i Python 3.11 lub 3.12 x64.
- Zgodny USB Bluetooth LE HCI adapter/dongle widoczny dla Bumble przez libusb.
- WinUSB przypisany do tego zewnętrznego adaptera. Nie podmieniaj sterownika wbudowanego Bluetooth: Windows może wtedy stracić zwykłą łączność.
- Random-static MAC własnego żetonu Triki.
- Obraz iPhone'a wyświetlony na monitorze Windows przez aplikację mirroringu.

To nie wymaga ESP32 ani programowalnej płytki, ale wymaga adaptera BLE dostępnego bezpośrednio przez USB HCI. Wbudowany adapter komputera może być zablokowany przez sterownik Windows.

## Instalacja

1. Rozpakuj ZIP repozytorium albo sklonuj je i otwórz `cmd.exe` w folderze projektu.
2. Utwórz środowisko i zainstaluj zależności:

```bat
py -3.11 -m venv .venv
.venv\Scripts\activate.bat
python -m pip install --upgrade pip
python -m pip install -r requirements-windows.txt
bumble-usb-probe
```

`bumble-usb-probe` musi pokazać USB HCI adapter. Jeśli nie pokazuje go, nie uruchamiaj Zadig na wbudowanym radiu. Zgodny zewnętrzny dongle może wymagać przypisania sterownika WinUSB przez Zadig; po tej zmianie Windows nie będzie używał go jako zwykłego adaptera.

## Odczyt MAC żetonu w Windows

Włącz żeton, zamknij bot, a w CMD uruchom skan:

```bat
bumble-scan usb:0
```

Znajdź reklamę `Triki DBD57D` (lub nazwę wydrukowaną na żetonie). Skopiuj adres z początku wpisu, tylko jeśli Bumble pokazuje typ `RANDOM` i dopisek `(static)`. Zakończ skan przez `Ctrl+C` przed uruchomieniem bota — ten sam dongle nie może jednocześnie skanować i reklamować. Jeśli skaner nie widzi kapsla, sprawdź, czy żeton jest wybudzony i czy używasz właściwego adaptera.

MAC jest identyfikatorem urządzenia: bot pyta o niego ukrytym promptem przy uruchomieniu. Nie dodawaj go do README, kodu ani publicznego commita.

## Uruchomienie

Otwórz program wyświetlający lustrzany ekran iPhone'a na monitorze. Uruchom:

```bat
python bot_windows.py --transport usb:0
```

Bot poprosi o MAC random-static kapsla bez wyświetlania wpisu. Obraz jest przetwarzany lokalnie. Domyślnie MSS przechwytuje monitor 1; inny monitor podaj przez `--monitor`, a wycinek przez `--region LEWO GÓRA SZEROKOŚĆ WYSOKOŚĆ`. Opóźnienie dopasuj parametrami `--latency-ms` i `--reaction-ms`. `q` lub `Esc` zamyka podgląd.

Po RX START bot wysyła ramki IMU 100 Hz w notyfikacjach `20/20/2`. Jeśli adapter nie daje dostępu HCI, GATT nie wystartuje. Samo użycie prawidłowego MAC nie gwarantuje akceptacji przez Żappkę — wymagany jest również zgodny adapter i obsługa reklamowania przez jego kontroler.