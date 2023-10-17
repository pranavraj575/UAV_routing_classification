function number_from_file(name)
    num=""
    for c in name
        if c in "0123456789"
            num=num*c
        end
    end
    return parse(Int32,num)
end