data = csvread('trajectory.csv', 1);

time = data(:, 1);

left_hip_angle = data(:, 2);
left_knee_spring_angle = data(:, 3);
left_toe_angle = data(:, 4);
right_hip_angle = data(:, 5);
right_knee_spring_angle = data(:, 6);
right_toe_angle = data(:, 7);

left_hip_qvel = data(:, 8);
left_knee_spring_qvel = data(:, 9);
left_toe_qvel = data(:, 10);
right_hip_qvel = data(:, 11);
right_knee_spring_qvel = data(:, 12);
right_toe_qvel = data(:, 13);

figure;
plot(time, left_hip_angle);
title('Time vs. Left Hip Angle');
xlabel('Time');
ylabel('Left Hip Angle');
print('Time vs. Left Hip Angle.png', '-dpng');

plot(time, left_knee_spring_angle);
title('Time vs. Left Knee Spring Angle');
xlabel('Time');
ylabel('Left Knee Spring Angle');
print('Time vs. Left Knee Spring Angle.png', '-dpng');

plot(time, left_toe_angle);
title('Time vs. Left Toe Angle');
xlabel('Time');
ylabel('Left Toe Angle');
print('Time vs. Left Toe Angle.png', '-dpng');

plot(time, right_hip_angle);
title('Time vs. Right Hip Angle');
xlabel('Time');
ylabel('Right Hip Angle');
print('Time vs. Right Hip Angle.png', '-dpng');

plot(time, right_knee_spring_angle);
title('Time vs. Right Knee Spring Angle');
xlabel('Time');
ylabel('Right Knee Spring Angle');
print('Time vs. Right Knee Spring Angle.png', '-dpng');

plot(time, right_toe_angle);
title('Time vs. Right Toe Angle');
xlabel('Time');
ylabel('Right Toe Angle');
print('Time vs. Right Toe Angle.png', '-dpng');

%-------------------------------------------------------------------------------

plot(time, left_hip_qvel);
title('Time vs. Left Hip Velocity');
xlabel('Time');
ylabel('Left Hip Velocity');
print('Time vs. Left Hip Velocity.png', '-dpng');

plot(time, left_knee_spring_qvel);
title('Time vs. Left Knee Spring Velocity');
xlabel('Time');
ylabel('Left Knee Spring Velocity');
print('Time vs. Left Knee Spring Velocity.png', '-dpng');

plot(time, left_toe_qvel);
title('Time vs. Left Toe Velocity');
xlabel('Time');
ylabel('Left Toe Velocity');
print('Time vs. Left Toe Velocity.png', '-dpng');

plot(time, right_hip_qvel);
title('Time vs. Right Hip Velocity');
xlabel('Time');
ylabel('Right Hip Velocity');
print('Time vs. Right Hip Velocity.png', '-dpng');

plot(time, right_knee_spring_qvel);
title('Time vs. Right Knee Spring Velocity');
xlabel('Time');
ylabel('Right Knee Spring Velocity');
print('Time vs. Right Knee Spring Velocity.png', '-dpng');

plot(time, right_toe_qvel);
title('Time vs. Right Toe Velocity');
xlabel('Time');
ylabel('Right Toe Velocity');
print('Time vs. Right Toe Velocity.png', '-dpng');
