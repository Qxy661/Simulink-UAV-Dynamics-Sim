function z = baro_sample(hTrue, sigma)
%BARO_SAMPLE 气压高度量测（向上为正）。

z = hTrue + sigma * randn();
end
