# PathPlanningProject
Программный проект для нахождения кратчайших путей при помощи алгоритмов Дейкстры или A*.

## Входные данные:
На вход подаётся XML-файл, содержащий опции алгоритма поиска, карту с препятствиями, координаты стартовой точки и цели, уровни логирования.

## Теги входного файла XML:

- _root_

- _map_
  - _title_ - заглавие
  - _width_ - ширина карты
  - _height_ - высота карты
  - _cellsize_ - масштаб карты
  - _startx_, _starty_ - координаты начала
  - _finishx_, _finishy_ - координаты конца
  - _grid_ - сама карта
 
- _algorithm_ - опции алгоритма
  - _metrictype_ - тип метрики
  - _searchtype_ - алгоритм поиска (Дейкстра или Астар)
  - _hweight_ - вес эвристической функции
  - _breakingties_ - g_max или g_min, при равенстве F-значений вершин сравниваются их g-значения
  - _allowdiagonal_ - разрешено ли движение по диагонали
  - _allowsqueeze_ - разрешено ли просачивание, то есть проход между двумя непроходимыми вершинами
  - _cutcorners_ - разрешено ли срезать углы
 
- _options_ - описание логирования
  - _loglevel_ - уровень логирования
  - _logpath_ - путь для сохранения выходного XML-файла
  - _logfilename_ - название выходного XML-файла
  
## Выходные данные:
На выходе мы получаем XML-файл, содержащий всю ту же информацию, что и входной XML-файл плюс следующая информация:

- _log_
  - _mapfilename_ - путь к исходному XML-файлу
  - Информация о работе алгоритма:
    - _summary time_ - суммарное время работы алгоритма
    - _length_ - длина пути
    - _nodescreated_ - количество созданных вершин
    - _numberofsteps_ - количество шагов в алгоритме
  - _path_ - путь от начало до конца или "Path NOT found!" в случае отсутствия пути
  - _lplevel_ - вершины, образующие путь
  - _hplevel_
## Начало работы
Клонируйте содержимое этого репозитория в нужную директорию при помощи Git bash'a. (В нужной директории Rightclick + Git Bash here)
```bash
git clone https://github.com/*account*/PathPlanningProject.git
```

### Сборка и запуск

Сборку проекта возможно осуществить двумя способами:
- Используя QtCreator и qmake;
- Используя CMake.

При использовании CMake сборка и запуск может производиться как из командной строки, так и при помощи различных IDE (например JetBrains CLion). Ниже приведены скрипты сборки и запуска с использованием командной строки.

### Linux и Mac
Release сборка:
```bash
cd PathPlanningProject
cd Build
cd Release
cmake ../../ -DCMAKE_BUILD_TYPE="Release"
make
make install
```

Debug сборка:
```bash
cd PathPlanningProject
cd Build
cd Debug
cmake ../../ -DCMAKE_BUILD_TYPE="Debug"
make
make install
```

Запуск:
```bash
cd ../../Bin/{Debug|Release}/
./PathPlanning ../../Examples/example.xml
```
Результат запуска:

![cmake_run](./Images/cmake1.png)

### Windows
Release сборка:
```cmd
cd PathPlanningProject
cd Build
cd Release
set PATH
cmake ../../ -DCMAKE_BUILD_TYPE="Release" -G "MinGW Makefiles"
mingw32-make
mingw32-make install
```

Debug сборка:
```cmd
cd PathPlanningProject
cd Build
cd Debug
set PATH
cmake ../../ -DCMAKE_BUILD_TYPE="Debug" -G "MinGW Makefiles"
mingw32-make
mingw32-make install
```

Запуск:
```cmd
cd ../../Bin/{Debug|Release}/
PathPlanning.exe ../../Examples/example.xml
```

Результат запуска:
![cmake_run2](./Images/cmake.png)

## Тестирование 
```
 ctest
```

либо (для более подробного вывода):
```
 ctest --output-on-failure
```
## Контакы
Хайбулин Даниэль
- https://t.me/kiDaniel
