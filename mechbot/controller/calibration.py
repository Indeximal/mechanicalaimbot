import numpy as np

from mechbot.utils.vector_utils import vec_len

# from mechbot.controller.simulator import MechanicalSimulator


class CalibrationHelper:
    def __init__(self, interface, initial_device, wait_duration=70,
                 phase_1_deflection=.5):
        self.interface = interface
        self.device = initial_device
        self.step_1 = 0
        self.step_2 = 0
        self.phase = 0
        self.subphase = 0
        self.done = False
        self.wait_duration = wait_duration
        self.phase_1_deflection = phase_1_deflection

    # def _loss(self, pos1, pos2):
    #     return vec_len(pos1 - pos2)

    def _gradient_descent_step(self, step1, step2, input_pos,
                               dx=.01, dphi=.05, dgap=.01, epsilon=0.005):
        # Function to maximize
        def calc_loss():
            predicted_pos = self.device.calculate_cords(step1, step2)
            return - vec_len(predicted_pos - input_pos)

        # Baseline loss
        loss_0 = calc_loss()

        # Calculate the change in Loss for each property numerically
        self.device.motor1.pos[0] += dx
        dL_x1 = calc_loss() - loss_0
        self.device.motor1.pos[0] -= dx

        self.device.motor1.pos[1] += dx
        dL_y1 = calc_loss() - loss_0
        self.device.motor1.pos[1] -= dx

        self.device.motor2.pos[0] += dx
        dL_x2 = calc_loss() - loss_0
        self.device.motor2.pos[0] -= dx

        self.device.motor2.pos[1] += dx
        dL_y2 = calc_loss() - loss_0
        self.device.motor2.pos[1] -= dx

        self.device.motor1.align += dphi
        dL_phi1 = calc_loss() - loss_0
        self.device.motor1.align -= dphi

        self.device.motor2.align += dphi
        dL_phi2 = calc_loss() - loss_0
        self.device.motor2.align -= dphi

        self.device.gap += dgap
        dL_gap = calc_loss() - loss_0
        self.device.gap -= dgap

        # Make a step in the direction of the gradient
        self.device.motor1.pos[0] += dL_x1 / dx * epsilon
        self.device.motor1.pos[1] += dL_y1 / dx * epsilon
        self.device.motor2.pos[0] += dL_x2 / dx * epsilon
        self.device.motor2.pos[1] += dL_y2 / dx * epsilon
        self.device.motor1.align += dL_phi1 / dphi * epsilon
        self.device.motor2.align += dL_phi2 / dphi * epsilon
        self.device.gap += dL_gap / dgap * epsilon

        return calc_loss()

    def _advance(self):
        self.phase += 1
        self.subphase = 0

    def _evaluate_phase_1(self):
        points = np.array(self.phase_1_points)
        print(points)
        h_lines = [np.polyfit(points[..., i, 0], points[..., i, 1], 1)
                   for i in range(3)]
        m1_x = [(h_lines[i][0] - h_lines[j][0]) / (h_lines[i][1] - h_lines[j][1])
                for i, j in [(0, 1), (1, 2), (2, 0)]]
        m1_y = np.array(m1_x) * \
            np.array(h_lines)[..., 1] + np.array(h_lines)[..., 0]
        m1_avg = np.array([np.average(m1_x), np.average(m1_y)])
        print(m1_avg)
        v_lines = [np.polyfit(points[i, ..., 0], points[i, ..., 1], 1)
                   for i in range(3)]

    def _next_step(self):
        input_pos = self.interface.get_input()
        deflection = vec_len(input_pos)

        if self.phase == 0:
            if deflection < self.phase_1_deflection:
                self.step_1 += 1
                self.step_2 += 1
            else:
                self._advance()
                self.phase_1_center_1 = self.step_1
                self.phase_1_center_2 = self.step_2
                self.phase_1_width = int(self.phase_1_center_1 / 3)
                self.phase_1_points = np.zeros((3, 3, 2))

        elif self.phase == 1:
            lastphase = self.subphase - 1
            if self.subphase > 0:
                x = int(lastphase // 3)
                y = (lastphase % 3)
                self.phase_1_points[x, y] = np.array(input_pos)

            x = int(self.subphase // 3)
            y = (self.subphase % 3)

            self.subphase += 1
            if self.subphase > 9:
                self._evaluate_phase_1()
                self._advance()
            else:
                self.step_1 = self.phase_1_center_1 + \
                    (x - 1) * self.phase_1_width
                self.step_2 = self.phase_1_center_2 + \
                    (y - 1) * self.phase_1_width

        else:
            self.done = True

    def tick(self, tick_number):
        if self.done:
            return

        if tick_number % self.wait_duration == 0:
            self._next_step()
            self.interface.cmd_goto(self.step_1, self.step_2)

    def is_done(self):
        return self.done

    def get_best_guess(self):
        return self.device
