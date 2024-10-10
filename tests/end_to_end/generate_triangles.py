import random 

tests_number = 10

for test_number in range(0, tests_number):
    name_of_file = "test_" + f'{test_number + 1:02}' + ".in"
    file = open(name_of_file, 'w')

    number_of_triangles = random.randint(100, 100)

    test_text = str(number_of_triangles) + "\n"

    for elem_number in range(number_of_triangles) :
        first_point  = random.normalvariate(0, 7)
        second_point = random.normalvariate(3, 40)
        third_point  = random.normalvariate(9, 100)
        if elem_number % 50 == 0:
            file.write(test_text)
            test_text = ""
        test_text += str(first_point)[:6] + " " + str(second_point)[:6] + " " + str(third_point)[:6] + " "
    test_text += '\n'
    file.write(test_text)
    file.close()
