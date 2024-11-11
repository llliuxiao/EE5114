function [node, new_queue] = heap_pop(queue, field_name)
    node = queue(1);
    if size(queue, 1) == 1
        new_queue = [];
        return;
    end
    new_queue = [queue(end); queue(2: end - 1)];
    new_queue = heap_adjust(1, new_queue, field_name);
end

function [new_queue] = heap_adjust(parent_index, queue, field_name)
    left_son_index = 2 * parent_index;
    right_son_index = 2 * parent_index + 1;
    queue_size = size(queue, 1);
    new_queue = queue;
    if left_son_index <= queue_size
        left_son_value = queue(left_son_index).(field_name);
        if right_son_index > queue_size
            min_value = left_son_value;
            min_index = left_son_index;
        else
            right_son_value = queue(right_son_index).(field_name);
            if left_son_value >= right_son_value
                min_value = right_son_value;
                min_index = right_son_index;
            else
                min_value = left_son_value;
                min_index = left_son_index;
            end
        end
        if min_value < queue(parent_index).(field_name)
            temp = queue(parent_index);
            new_queue(parent_index) = queue(min_index);
            new_queue(min_index) = temp;
            new_queue = heap_adjust(min_index, new_queue, field_name);
        end
    end

end