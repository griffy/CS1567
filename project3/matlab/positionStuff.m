matrixes = rightRegLine;

j=0;
sum = zeros(10,1);
for i = 0:9;
    count=0;
    for j = 1:25;
        if (matrixes(i*25+j) == -999.0 || isnan(matrixes(i*25+j)) || matrixes(i*25+j) == 0)
            %%display('error');
        else
            %display(matrixes(i*15+j));
            sum(i+1)=sum(i+1)+matrixes(i*25+j);
            count=count+1;
        end
    end
    display(sum(i+1)/count);
end