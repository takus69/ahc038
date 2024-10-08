use proconio::{input, marker::Chars};
use std::collections::VecDeque;

struct RobotArm {
    n: usize,
    root: (i8, i8),  // 根の位置
    leaf: (i8, i8),  // 葉の位置
    long: usize,  // アームの最大長
    nodes: Vec<usize>,  // 各ノードの状態(親との相対回転状態)
    v2: usize,
    p_l: Vec<(usize, usize)>,
}

impl RobotArm {
    fn new(n: usize, (x, y): (i8, i8), v: usize) -> RobotArm {
        let root = (x, y);
        let mut nodes: Vec<usize> = vec![0; v-1];
        nodes[0] = 1;
        let leaf = (x, y+v as i8-1);
        let v2 = v;
        let mut p_l: Vec<(usize, usize)> = Vec::new();
        for p in 0..nodes.len() {
            p_l.push((p, 1));
        }

        RobotArm { n, root, leaf, long: v-1, nodes, v2, p_l }
    }

    fn dist(&self, (x, y): (i8, i8), (x2, y2): (i8, i8)) -> usize {
        ((x-x2).abs() + (y-y2).abs()) as usize
    }

    fn add(&self, (x, y): (i8, i8), (dx, dy): (i8, i8)) -> (i8, i8) {
        (x + dx, y + dy)
    }

    fn is_same_dir(&self, dir: (i8, i8), now: (i8, i8), target: (i8, i8)) -> bool {
        let dir2 = (target.0 - now.0, target.1 - now.1);
        dir.0*dir2.0 > 0 || dir.1*dir2.1 > 0
    }

    fn dot(&self, dir: (i8, i8), now: (i8, i8), target: (i8, i8)) -> i8 {
        let dir2 = (target.0 - now.0, target.1 - now.1);
        dir.0*dir2.0 + dir.1*dir2.1
    }

    fn r#move(&mut self, (tx, ty): (i8, i8)) -> Vec<String> {
        let mut ops: Vec<Vec<char>> = Vec::new();
        let DIRS = [(-1, 0), (0, 1), (1, 0), (0, -1)];
        
        // 葉が目的地に行くまで繰り返す
        let mut cnt = 0;
        while self.leaf != (tx, ty) {
            cnt += 1;
            // if cnt > 20 { break; }
            // println!("move arm: root: {:?}, leaf: {:?}, target: {:?}", self.root, self.leaf, (tx, ty));
            let mut op: Vec<char> = Vec::new();

            // (tx, ty)-rootがアームの最大長より長い場合、もしくは、アームの最大長と偶奇が一致していない場合は移動する
            let diff = self.dist(self.root, (tx, ty));
            // println!("long: {}, diff: {}, root: {:?}, target: {:?}", self.long, diff, self.root, (tx, ty));
            if diff > self.long || diff%2 != self.long%2 || self.root == (tx, ty) {
                for d in 0..4 {
                    let tmp_root = self.add(self.root, DIRS[d]);
                    if tmp_root.0 < 0 || tmp_root.0 >= self.n as i8 || tmp_root.1 < 0 || tmp_root.1 >= self.n as i8 { continue; }  // rootが枠外の場合は不採用
                    let tmp_diff = self.dist((tx, ty), tmp_root);
                    let mut ok = false;  // 移動を採用するかのフラグ
                    if diff > self.long && diff > tmp_diff { ok = true; }
                    if self.long >= tmp_diff { ok = true; }
                    if ok {
                        let o_s = match d {
                            0 => 'U',
                            1 => 'R',
                            2 => 'D',
                            3 => 'L',
                            _ => '.',
                        };
                        op.push(o_s);
                        self.root = tmp_root;
                        break;
                    }
                }
            } else {
                op.push('.');
            }
            // println!("move root: {:?}, {:?}", self.root, op);

            // アームの葉を(tx, ty)に移動する。小さいノードから目的地に近くなる回転を行う
            let mut now = self.root;
            let mut dir = 0;
            // アームの最大長より短い場合は、寄り道を早い段階でする
            let diff = self.dist(self.root, (tx, ty));
            let mut leave_cnt = if self.long > diff { (self.long - diff)/2 } else { 0 };
            // println!("leave_cnt: {}", leave_cnt);
            for i in 0..self.long {
                dir = (dir + self.nodes[i]) % 4;  // 相対の方向から絶対の方向に変換
                // println!("target: ({}, {}), now: {:?}, diff: {}, dir: {}, {:?}", tx, ty, now, diff, dir, DIRS[next_dir]);
                let mut eval = 0;
                let mut o= 0;
                let mut next_dir = dir;
                let mut leave = false;
                for r in [0, 1, 3] {
                    let tmp_dir = (dir + r) % 4;
                    let mut tmp_leave = false;
                    // 方向の重みづけ
                    let mut tmp_eval = match (self.nodes[i] + r) % 4 {
                        0 => 30,
                        1 => 20,
                        2 => 10,
                        3 => 20,
                        _ => 0,
                    };
                    if i == 0 {
                        let dot = self.dot(DIRS[tmp_dir], now, (tx, ty));
                        tmp_eval = if dot > 0 { 30 } else if dot < 0 { 10 } else { 20 };
                    }  // 根は方向の重みはなし
                    let is_same_dir = self.is_same_dir(DIRS[tmp_dir], now, (tx, ty));
                    // 寄り道を優先
                    if leave_cnt > 0 && !is_same_dir {
                        tmp_eval += 30;
                        tmp_leave = true;
                    } else if !is_same_dir {  // 目的地に向かわない場合は向きだけ考える
                        tmp_eval /= 10;
                    }
                    // println!("i: {}, eval: {}, tmp_eval: {}, r: {}, tmp_dir: {}", i, eval, tmp_eval, r, tmp_dir);

                    if tmp_eval > eval {
                        o = r;
                        next_dir = tmp_dir;
                        eval = tmp_eval;
                        leave = tmp_leave;
                    }
                    // println!("i: {}, eval: {}, o: {}, next_dir: {}", i, eval, o, next_dir);
                }
                self.nodes[i] = (self.nodes[i] + o) % 4;
                let o_s = match o {
                    0 => { '.' },
                    1 => { 'R' },
                    3 => { 'L' },
                    _ => { '.' },
                };
                op.push(o_s);
                now = self.add(now, DIRS[next_dir]);
                dir = next_dir;
                if leave { leave_cnt -= 1; }
            }
            self.leaf = now;
            for _ in 0..=self.long { op.push('.'); }
            // println!("move leaf: {:?}, {:?}", self.leaf, op);
            // println!("nodes: {:?}", self.nodes);
            ops.push(op);
        }

        let mut ret: Vec<String> = Vec::new();
        for (i, op) in ops.iter().enumerate() {
            let mut op = op.clone();
            if i == ops.len()-1 {
                let op_len = op.len();
                op[op_len-1] = 'P';
            }
            ret.push(op.iter().map(|x| x.to_string()).collect::<Vec<String>>().join(""));
        }
        ret
    }
}

struct Solver {
    n: usize,
    m: usize,
    v: usize,
    s: Vec<Vec<bool>>,
    t: Vec<Vec<bool>>,
    v2: usize,
    p_l: Vec<(usize, usize)>,
    x : i8,
    y: i8,
    op: Vec<String>,
}

impl Solver {
    fn solve(&mut self) {
        // たこ焼きの処理順を決定
        let mut t_tako: Vec<(i8, i8)> = Vec::new();
        for j in 0..self.n {
            if j%2 == 0 {
                for i in 0..self.n {
                    if self.t[i][j] { t_tako.push((i as i8, j as i8)); }
                }
            } else {
                for i in (0..self.n).rev() {
                    if self.t[i][j] { t_tako.push((i as i8, j as i8)); }
                }
            }
        }

        // アームの初期化
        (self.x, self.y) = t_tako[0];
        let mut arm = RobotArm::new(self.n, (self.x, self.y), self.v);
        self.v2 = arm.v2;
        self.p_l.clone_from(&arm.p_l);

        // 処理
        let mut fixed: Vec<Vec<bool>> = vec![vec![false; self.n]; self.n];  // 配置が確定したたこ焼き
        for i in 0..self.m {
            let target = t_tako[i];
            if self.s[target.0 as usize][target.1 as usize] {
                fixed[target.0 as usize][target.1 as usize] = true;
                continue;
            }
            let start = self.decide_start(&target, &fixed);
            // println!("start: {:?}, target: {:?}", start, target);
            let ops = arm.r#move(start);
            self.op.extend(ops);
            if start == arm.leaf {
                self.s[start.0 as usize][start.1 as usize] = false;
            }
            // println!("get: {:?}", arm.leaf);
            let ops = arm.r#move(target);
            self.op.extend(ops);
            if target == arm.leaf {
                self.s[target.0 as usize][target.1 as usize] = true;
                fixed[target.0 as usize][target.1 as usize] = true;
            }
            // println!("put: {:?}", arm.leaf);
        }
        
    }

    fn decide_start(&self, target: &(i8, i8), fixed: &[Vec<bool>]) -> (i8, i8) {
        let mut dist = usize::MAX;
        let (mut x, mut y) = (0, 0);
        for i in 0..self.n {
            for j in 0..self.n {
                if !self.s[i][j] { continue; }
                if self.t[i][j] || fixed[i][j] { continue; }
                let tmp_dist = self.dist(target, &(i as i8, j as i8));
                if dist > tmp_dist {
                    dist = tmp_dist;
                    (x, y) = (i, j);
                }
            }
        }

        (x as i8, y as i8)
    }

    fn dist(&self, p1: &(i8, i8), p2: &(i8, i8)) -> usize {
        (p1.0.abs_diff(p2.0) + p1.1.abs_diff(p2.1)) as usize
    }

    fn center(&self, s: &Vec<Vec<bool>>) -> (i8, i8) {
        let (mut c_x, mut c_y) = (0, 0);
        for x in 0..self.n {
            for y in 0..self.n {
                if s[x][y] {
                    c_x += x;
                    c_y += y;
                }
            }
        }
        c_x /= self.m;
        c_y /= self.m;

        (c_x as i8, c_y as i8)
    }

    fn ans(&self) {
        println!("{}", self.v2);
        for (p, l) in self.p_l.iter() {
            println!("{} {}", p, l);
        }
        println!("{} {}", self.x, self.y);
        for s in self.op.iter() {
            println!("{}", s);
        }

        // スコア出力
        let mut correct = 0;
        for i in 0..self.n {
            for j in 0..self.n {
                if self.t[i][j] && self.s[i][j] {
                    correct += 1;
                }
            }
        }
        let score = if correct == self.m { self.op.len() } else { 100000 + 1000*(self.m - correct)};
        eprintln!("{{ \"N\": {}, \"M\": {}, \"V\": {}, \"score\": {} }}", self.n, self.m, self.v, score);
    }
}

fn parse_input() -> Solver {
    input! {
        n: usize,
        m: usize,
        v: usize,
        s: [Chars; n],
        t: [Chars; n],
    }

    let s: Vec<Vec<bool>> = s.iter()
        .map(|inner_vec| {
            inner_vec.iter()
                .map(|&c| c == '1')
                .collect()
        })
        .collect();
    let t: Vec<Vec<bool>> = t.iter()
        .map(|inner_vec| {
            inner_vec.iter()
                .map(|&c| c == '1')
                .collect()
        })
        .collect();
    let v2 = 0;
    let p_l: Vec<(usize, usize)> = Vec::new();
    let (x, y) = (0, 0);
    let op: Vec<String> = Vec::new();

    Solver {n, m, v, s, t, v2, p_l, x, y, op }
}

fn main() {
    let mut solver = parse_input();
    solver.solve();
    solver.ans();
}

// tests
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_rotob_arm() {
        let n = 10;
        let (x, y) = (0, 0);
        let v = 4;
        let mut arm = RobotArm::new(n, (x, y), v);
        assert_eq!(arm.root, (0, 0));
        assert_eq!(arm.leaf, (0, 3));
        let mut ops = arm.r#move((1, 2));
        assert_eq!(arm.leaf, (1, 2));
        assert_eq!(ops.len(), 1);
        let mut ops = arm.r#move((3, 0));
        assert_eq!(arm.leaf, (3, 0));
        assert_eq!(ops.len(), 1);
        let mut ops = arm.r#move((6, 0));
        assert_eq!(arm.leaf, (6, 0));
        println!("end (6, 0)");
        let mut ops = arm.r#move((0, 0));
        println!("end (0, 0)");
        assert_eq!(arm.leaf, (0, 0));
    }

}
