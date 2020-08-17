use components::{data_types::Pattern, impls::MazeBuilder};

use crate::alias::{Maze, MazeWidth};

fn costs(pattern: Pattern) -> u16 {
    0
}

pub fn init_maze() -> Maze {
    MazeBuilder::new()
        .costs(costs as fn(Pattern) -> u16)
        .build::<MazeWidth>()
}
