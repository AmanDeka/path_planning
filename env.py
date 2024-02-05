class Env:
    def __init__(self):
        self.x_range = (0, 50)
        self.y_range = (0, 30)
        self.obs_boundary = self.obs_boundary()
        self.obs_circle = []
        self.obs_rectangle = self.obs_rectangle()

    @staticmethod
    def obs_boundary():
        obs_boundary = [
            [0, 0, 1, 30],
            [0, 30, 50, 1],
            [1, 0, 50, 1],
            [50, 1, 1, 30]
        ]
        return obs_boundary

    @staticmethod
    def obs_rectangle():
        obs_rectangle = [
            [14, 12, 8, 2],
            [18, 22, 8, 3],
            [26, 7, 2, 12],
            [32, 14, 10, 2]
        ]
#bottom left (x,y) coordinates, width and height
        rect_1 = [ [14, 12, 10, 4],
                [18, 21, 10, 3],
                [26, 7, 3, 12],
                [32, 14, 12, 2]]
        rect_2 = [ [16, 12, 7, 3],
                [18, 20, 8, 3],
                [27, 6, 2, 10],
                [30, 11, 10, 2]]
        rect_3 = [ [12, 15, 9, 3],
                [21, 25, 7, 2],
                [26, 10, 3, 10],
                [33, 14, 7, 3],
                [23,5,3,4]]
        rect_4 = [ [14, 12, 10, 4],
                [20, 22, 8, 3],
                [27,5,4,16],
                [34, 13, 10, 4]]
        return rect_4

    @staticmethod
    def obs_circle():
        obs_cir = [
            [7, 12, 3],
            [46, 20, 2],
            [15, 5, 2],
            [37, 7, 3],
            [37, 23, 3]
        ]

        return obs_cir