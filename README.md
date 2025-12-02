# seminarsU2035

### Создание оркужения python для запуска кода:

1. Создаем среду:

```bash
python3 -m venv seminar_env
```

2. Активируем её:

```bash
source ./seminar_env/bin/activate
```

3. Скачиваем в неё библиотеки:

```bash
pip install -r req.txt
```

## !!! Запуск нужной карты в симуляторе

1. Нужно находиться в корневой папке, т.е. в папке seminarU2035
2. Из этой папки пишем в терминал:
```bash
./start_sim.sh <номер семинара>
```

Например, для семинара 2 "Обработка сигналов и изображений" нужно написать в терминал:

```bash
./start_sim.sh 2
```

3. Чтобы запустить мост для ROS пишем в другом терминале:

```bash
source /opt/ros/humble/setup.bash

./start_ros.sh
```

### Для запуска примеры карты с экзамена:

```bash
./start_exam_map.sh
```

```bash
source /opt/ros/humble/setup.bash

./start_ros.sh
```
