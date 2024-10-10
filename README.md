## Triangls

## Исследование пересечения треугольников в трехмерном пространстве

Расположить треугольники в пространстве можно **4** различными способами:

![image](triangles_and_planes.png)

# Алгоритм для исследования пересечения треугольников в 3D:

0. Проверить треугольники на вырожденность, если они не являются точками и отрезками, то получить уравнение плоскостей, в которых лежат треугольники, иначе исследовать взаимное расположение точек и отрезков.

1. Если плоскости параллельны, то необходимо узнать расстояние **d** между ними.
1.1 Если **d** отлично от 0, то треугольники **не пересекаются**.
1.2 Если **d** равно 0, то нужно исследовать пересечение 2D треугольников. 
(Если плоскости параллельны, то можно определить их совпадение подстановкой координат любой точки первой плоскости в уравнение второй плоскости. Если соотоношение **Ax + By + Cz + D = 0** выполнится, то плоскости совпадают.)
Чтобы определить перескаются ли трегольники, когда находятся в одной плоскости, нужно проверить два условия.
Во-первых, возможна ситауция, при которой один треугольник лежит внутри другого. Для этого можно проверять то, что каждая вершина одного трегольника лежит внутри другого. Это можно проверить с помощью свойств векторного произведения.
Во-вторых, отрезки, из которых состоит первый треугольник, могут пересекать второй треугольник. Поэтому нужно проверить на пересечение все отрезки треугольников. 
Если выполняется первое или второе условие, то трегольники **пересекаются**.
Если оба условия не выполняются, то треугольники **не пересекаются**.

2. Если плоскости не являются параллельными, то:
2.1 Исследовать **знаковые** расстояния от вершин одного труегольника, до плоскости другого. Если расстояния одного знака, то треугольники **не пересекаются**
2.2 Иначе треугольники могут пересекаться по линии пересечения их плоскостей. 
В этом случае нужно найти прямую пересечения двух плоскостей, задаваемых треугольниками.
Нужно наложить на эту прямую интервалы её пересечения двумя треугольниками.
Если **интервалы пересекаются**, то треугольники **пересекаются**.
Если **интервалы не пересекаются**, то треугольники **не пересекаются**.

## Оптимизация 
Чтобы проверить какие из треугольников пересекаются, необходимо **O(N^2)** проверок *(это много)*. Чтобы оптимизировать алгоритм можно отказаться от простого перебора и использовать **октодеревья**.
Если создать фигуру, которая ограничит пространство (bounding box), например параллелепипед, и разделять его на две части, в которых и будут полностью содержаться треугольники, то можно сократить количество проверок (то есть не проверять на пересечение треугольники, которые содержатся в двух разных половинах параллепипеда).

Эту идею можно развить. Можно с помощью дерева устроить разбиение пространства такое, что в каждом узле дерева будет только один объект (критерий узла в октодереве). При это некоторые узлы могут оказаться пустыми. Также во всех узлах, находящихся на одинаковом уровне, не будет пересекающихся фигур.

Свое октодерево я строил следующим образом:
1. Каждый узел содержит указатель на родительский
2. Каждый узел list с треугольниками, которые содержатся в данном узле
3. Каждый узел содержит свою центральную точку и размеры по осям X, Y, Z
4. Каждый узел содержит массив указателей на своих потомков и массив, содержащий информацию о том, какие из них используются

Такая оптимизация заметно уменьшает время поиска пересекающихся треугольников.

# Использование 

## Сборка проекта:
```cmake -B build```

## Чтобы запустить программу:
```cd build```
```./triangles```

## Чтобы запустить unit-тесты:
```cd build```
```cd tests```
```./unit_test```

## end to end тесты:
```cd tests```
```cd end_to_end```
```./end_to_end_testing.sh```

## Чтобы cгенеровать 10 тестов:
```python3 generate_triangles.py```