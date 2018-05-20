%%
f = fopen('qlog.bin');
data = fread(f, '*double');
fclose(f);

nq = 1 + 35 + 32;
n = floor(numel(data) / nq) * nq;
data = data(1:n);
qdata = reshape(data, nq, []);

t = qdata(1, :);
qpos = qdata(2:36, :);
qvel = qdata(37:end, :);

plot(t, qpos(1:6, :))

%%

f = fopen('structs.bin');
bytes = fread(f, '*uint8');
fclose(f);

load logpack.mat

nb = 697 + 58;
n = floor(numel(bytes) / nb) * nb;
bytes = bytes(1:n);
bytes = reshape(bytes, nb, []);
obj.logBytes = bytes(:, 1:numel(t));

torques = getField(obj, 'userIn.torque');
mpos = double([...
  getField(obj, 'out.leftLeg.hipRollDrive.position')
  getField(obj, 'out.leftLeg.hipYawDrive.position')
  getField(obj, 'out.leftLeg.hipPitchDrive.position')
  getField(obj, 'out.leftLeg.kneeDrive.position')
  getField(obj, 'out.leftLeg.footDrive.position')
  getField(obj, 'out.rightLeg.hipRollDrive.position')
  getField(obj, 'out.rightLeg.hipYawDrive.position')
  getField(obj, 'out.rightLeg.hipPitchDrive.position')
  getField(obj, 'out.rightLeg.kneeDrive.position')
  getField(obj, 'out.rightLeg.footDrive.position')]);
mvel = double([...
  getField(obj, 'out.leftLeg.hipRollDrive.velocity')
  getField(obj, 'out.leftLeg.hipYawDrive.velocity')
  getField(obj, 'out.leftLeg.hipPitchDrive.velocity')
  getField(obj, 'out.leftLeg.kneeDrive.velocity')
  getField(obj, 'out.leftLeg.footDrive.velocity')
  getField(obj, 'out.rightLeg.hipRollDrive.velocity')
  getField(obj, 'out.rightLeg.hipYawDrive.velocity')
  getField(obj, 'out.rightLeg.hipPitchDrive.velocity')
  getField(obj, 'out.rightLeg.kneeDrive.velocity')
  getField(obj, 'out.rightLeg.footDrive.velocity')]);

plot(t(10:end), torques(:, 10:end))

%%

p = 0.841;
a = 14816;
b = 28271 - 5*p/5e-4;
p = 0.841;

tm = mod(t(a:b) - t(a), p);

plot(tm, qpos(1:6, a:b))

axis([0 p 0.99 1.05])

%%

torques2 = zeros(10, 1682);
mpos2 = zeros(10, 1682);
mvel2 = zeros(10, 1682);
qpos2 = zeros(35, 1682);
qvel2 = zeros(32, 1682);
t2 = tm(1:1682);

for i = 0:2
  range = a+i*1682:a+(i+1)*1682-1;
    torques2 = torques2 + torques(:, range) / 3;
    mpos2 = mpos2 + mpos(:, range) / 3;
    mvel2 = mvel2 + mvel(:, range) / 3;
    qpos2 = qpos2 + qpos(:, range) / 3;
    qvel2 = qvel2 + qvel(:, range) / 3;
end

w = linspace(0, 1, 1682);
qpos2(2:end, :) = qpos2(2:end, :) - w .* (qpos2(2:end, end) - qpos2(2:end, 1));
qvel2 = qvel2 - w .* (qvel2(:, end) - qvel2(:, 1));
torques2 = torques2 - w .* (torques2(:, end) - torques2(:, 1));
mpos2 = mpos2 - w .* (mpos2(:, end) - mpos2(:, 1));
mvel2 = mvel2 - w .* (mvel2(:, end) - mvel2(:, 1));

qpos2(1, :) = qpos2(1, :) - qpos2(1, 1);
qpos2(2, :) = qpos2(2, :) - mean(qpos2(2, :));


plot([t2, t2 + t2(end) + 5e-4], [qpos2(1:7, :), qpos2(1:7, :)])
plot([t2, t2 + t2(end) + 5e-4], [qvel2(1:6, :), qvel2(1:6, :)])
plot([t2, t2 + t2(end) + 5e-4], [torques2, torques2])


%%
qdata2 = [t2; qpos2; qvel2; torques2; mpos2; mvel2];
f = fopen('stepdata.bin', 'wb');
fwrite(f, qdata2, 'double');
