import random

tests_number = 10

for test_number in range(tests_number):
    name_of_file = f"test_{test_number + 1:02}.in"
    
    with open(name_of_file, 'w') as file:
        number_of_triangles = 10

        test_text = str(number_of_triangles) + "\n"

        for elem_number in range(number_of_triangles):

            first_point  = (random.randint(-100, 100), random.randint(-100, 100), random.randint(-100, 100))
            second_point = (random.randint(-100, 100), random.randint(-100, 100), random.randint(-100, 100))
            third_point  = (random.randint(-100, 100), random.randint(-100, 100), random.randint(-100, 100))

            formatted_triangle = f"{first_point[0]} {first_point[1]} {first_point[2]} " f"{second_point[0]} {second_point[1]} {second_point[2]} " f"{third_point[0]} {third_point[1]} {third_point[2]}\n"
            test_text += formatted_triangle

        file.write(test_text)