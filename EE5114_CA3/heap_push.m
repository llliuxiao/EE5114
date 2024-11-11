function [new_queue] = heap_push(queue, node, field_name)
    new_queue = [queue; node];
    son_index = size(new_queue, 1);
    parent_index = floor(son_index / 2);
    while son_index > 1
        if new_queue(parent_index).(field_name) <= new_queue(son_index).(field_name)
            break
        end
        temp = new_queue(son_index);
        new_queue(son_index) = new_queue(parent_index);
        new_queue(parent_index) = temp;
        son_index = parent_index;
        parent_index = floor(son_index / 2);
    end
end