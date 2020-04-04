function y = fuzzy(fis,x)
    %#codegen
    opt = evalfisOptions('NumSamplePoints',51);
    y = evalfis(fis,x,opt);
end
