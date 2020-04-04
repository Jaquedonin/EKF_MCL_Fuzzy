fisObject = readfis("controlador_fuzzy2.fis");
fis = getFISCodeGenerationData(fisObject);
codegen('fuzzy','-args',{coder.Constant(fis),[0 0 0]},'-config:mex')