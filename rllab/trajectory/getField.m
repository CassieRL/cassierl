function out = getField(obj, field)

i = find(obj.fieldNames == field, 1);

inds = obj.startInds(i):obj.endInds(i);
bytes = obj.logBytes(inds, :);
out = typecast(bytes(:), char(obj.numTypes(i)));

ncols = size(obj.logBytes, 2);
nrows = numel(out) / ncols;
out = reshape(out, nrows, ncols);
