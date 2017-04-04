function [ f, n ] = calculateTorques( rotationMatrices, jointVelocities, jointAccelerations, jointMasses, centersOfMass1, centersOfMass2, jointMoments )
% Symbolically calculate the torque as a function of joint position,
% velocity, and acceleration

% centersOfMass1 is the centers of mass of a joint, viewed from the current
% joint, 2 is the center of mass viewed from the previous joint

jointNum = size(rotationMatrices, 3);
zvector = [0; 0; 1];
yvector = [0; 1; 0];
syms gravity;

frequency = sym(zeros(3, jointNum));
frequencyderivative = sym(zeros(3, jointNum));
vDerivative = sym(zeros(3, jointNum));
Vc = sym(zeros(3, jointNum));
F = sym(zeros(3, jointNum));
N = sym(zeros(3, jointNum));
f = sym(zeros(3, jointNum));
n = sym(zeros(3, jointNum));

frequency(:, 1) = jointVelocities(1)*zvector;
frequencyderivative(:, 1) = jointAccelerations(1)*zvector;
vDerivative(:, 1) = rotationMatrices(:, :, 1)*yvector*gravity;
Vc(:, 1) = cross(frequencyderivative(:, 1), centersOfMass1(:, 1)) + cross(frequency(:, 1), cross(frequency(:, 1), centersOfMass1(:, 1))) + vDerivative(:, 1);
F(:, 1) = jointMasses(1)*Vc(:, 1);
N(:, 1) = jointMoments(1)*frequencyderivative(:, 1) + cross(frequency(:, 1), (jointMoments(1)*frequency(:, 1)));

for i = 1:jointNum-1
    frequency(:, i+1) = rotationMatrices(:, :, i+1)*frequency(:, i) + jointVelocities(i+1)*zvector;
    frequencyderivative(:, i+1) = rotationMatrices(:, :, i+1)*frequencyderivative(:, i) + cross(rotationMatrices(:, :, i+1)*frequency(:, i),jointVelocities(i+1)*zvector) + jointAccelerations(i+1)*zvector;
    vDerivative(:, i+1) = rotationMatrices(:, :, i+1)*(cross(frequencyderivative(:, i), centersOfMass2(:, i+1))+cross(frequency(:, i), cross(frequency(:, i), centersOfMass2(:, i+1))) + vDerivative(:, i));
    Vc(:, i+1) = cross(frequencyderivative(:, i+1), centersOfMass1(:, i+1)) + cross(frequency(:, i+1), cross(frequency(:, i+1), centersOfMass1(:, i+1))) + vDerivative(:, i+1);
    F(:, i+1) = jointMasses(i+1)*Vc(:, i+1);
    N(:, i+1) = jointMoments(i+1)*frequencyderivative(:, i+1) + cross(frequency(:, i+1), (jointMoments(i+1)*frequency(:, i+1)));
end

f(:, end) = F(:, end);
n(:, end) = N(:, end) + cross(centersOfMass1(:, end), F(:, end));
for i = jointNum-1:-1:1
    f(:, i) = rotationMatrices(:, :, i+1).' * F(:, i+1) + F(:, i);
    n(:, i) = N(:, i) + rotationMatrices(:, :, i+1).' * n(:, i+1) + cross(centersOfMass1(:, i), F(:, i)) + cross(centersOfMass2(:, i+1), rotationMatrices(:, :, i+1).' * F(:, i+1));
end
end

