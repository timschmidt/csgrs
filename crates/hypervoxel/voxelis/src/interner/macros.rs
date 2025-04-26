#[macro_export]
macro_rules! get_next_index_macro {
    ($self:expr) => {{
        if let Some(index) = $self.free_indices.pop() {
            #[cfg(feature = "debug_trace_ref_counts")]
            println!("get_next_index: Recycled index: {}", index);

            #[cfg(feature = "memory_stats")]
            {
                $self.stats.alive_nodes += 1;
                $self.stats.max_alive_nodes =
                    $self.stats.max_alive_nodes.max($self.stats.alive_nodes);
                $self.stats.recycled_nodes -= 1;
                $self.stats.total_allocations += 1;
            }

            index
        } else if $self.next_index < $self.capacity as u32 {
            let index = $self.next_index;

            #[cfg(feature = "debug_trace_ref_counts")]
            println!("get_next_index: New index: {}", index);

            $self.next_index += 1;

            #[cfg(feature = "memory_stats")]
            {
                $self.stats.alive_nodes += 1;
                $self.stats.max_alive_nodes =
                    $self.stats.max_alive_nodes.max($self.stats.alive_nodes);
                $self.stats.allocated_nodes += 1;
                $self.stats.total_allocations += 1;
                $self.stats.max_node_id = $self.stats.max_node_id.max(index as usize);
            }

            index
        } else {
            panic!("Out of memory");
        }
    }};
}
