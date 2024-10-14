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

    fn next_node_dir_is_ok(&self, dir: usize, i: usize, now: (i8, i8), target: (i8, i8)) -> bool {
        // 折りたたんでいる場合に、目的地に近づけない場合に、次のノードが近づく方を選択するための処理
        let DIRS = [(-1, 0), (0, 1), (1, 0), (0, -1)];
        if i >= self.nodes.len()-1 {
            false
        } else {
            let next_node_dir = (dir + self.nodes[i+1])%4;
            self.is_same_dir(DIRS[next_node_dir], now, target)
        }
    }

    fn r#move(&mut self, (tx, ty): (i8, i8), next: &(i8, i8)) -> Vec<String> {
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
            let next_diff = self.dist(self.root, *next);
            let mut opt_eval = 0;
            let mut opt_root = self.root;
            let mut opt_o_s = '.';
            // println!("long: {}, diff: {}, root: {:?}, target: {:?}", self.long, diff, self.root, (tx, ty));
            if diff > self.long || diff%2 != self.long%2 || self.root == (tx, ty) {
                for d in 0..4 {
                    let tmp_root = self.add(self.root, DIRS[d]);
                    if tmp_root.0 < 0 || tmp_root.0 >= self.n as i8 || tmp_root.1 < 0 || tmp_root.1 >= self.n as i8 { continue; }  // rootが枠外の場合は不採用
                    let tmp_diff = self.dist((tx, ty), tmp_root);
                    let mut eval = if diff > self.long && diff > tmp_diff { 10 } else { 0 };
                    eval = if self.long >= tmp_diff { 10 } else { eval };
                    if self.dist(tmp_root, *next) < next_diff { eval *= 2; }
                    if eval > opt_eval {
                        opt_eval = eval;
                        opt_root = tmp_root;
                        opt_o_s = match d {
                            0 => 'U',
                            1 => 'R',
                            2 => 'D',
                            3 => 'L',
                            _ => '.',
                        };
                    }
                }
                op.push(opt_o_s);
                self.root = opt_root;
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
                        1 => 20 + if self.next_node_dir_is_ok(tmp_dir, i, self.add(now, DIRS[tmp_dir]), (tx, ty)) { 5 } else { 0 },
                        2 => 10,
                        3 => 20 + if self.next_node_dir_is_ok(tmp_dir, i, self.add(now, DIRS[tmp_dir]), (tx, ty)) { 5 } else { 0 },
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

#[derive(Clone)]
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
        // たこ焼きと目的地の場所
        let mut tako: Vec<(i8, i8)> = self.pos(&self.s);
        let dest: Vec<(i8, i8)> = self.pos(&self.t);

        // アームの初期化
        let (s_cm, s_cnt) = self.center(&self.s, &self.t);
        let s_cm = (s_cm.0 as i8, s_cm.1 as i8);
        (self.x, self.y) = s_cm;
        let mut arm = RobotArm::new(self.n, (self.x, self.y), self.v);
        self.v2 = arm.v2;
        self.p_l.clone_from(&arm.p_l);

        // 処理
        for _ in 0..self.m {
            // 重心を算出
            let (s_cm, s_cnt) = self.center(&self.s, &self.t);
            let s_cm = (s_cm.0 as i8, s_cm.1 as i8);
            if s_cnt == 0 { break; }
            let (t_cm, t_cnt) = self.center(&self.t, &self.s);
            let t_cm = (t_cm.0 as i8, t_cm.1 as i8);
            let dir_st = self.dir(&s_cm, &t_cm);

            // 次のたこ焼きを決定
            let mut next_tako_i = 0;
            let mut next_tako = tako[next_tako_i];
            let mut opt_eval = f64::MIN;
            for i in 0..self.m {
                // コスト算出
                // たこ焼きまでの移動
                let dist = self.dist(&arm.root, &tako[i]);
                let mut cost = (if dist > arm.long { dist - arm.long } else { 0 }).max(2);
                // たこ焼きから目的地の重心までの移動
                let dist = self.dist(&tako[i], &t_cm);
                cost += (if dist > arm.long { dist - arm.long } else { 0 }).max(2);

                // 利益算出
                let (x, y) = tako[i];
                let benefit = if !self.t[x as usize][y as usize] { self.each_debt(&tako[i], &s_cm, &dir_st) } else { 0 };

                let eval = benefit as f64 / cost as f64;
                // println!("i: {}, tako: {:?}, eval: {}, benefit: {}, cost: {}", i, tako[i], eval, benefit, cost);
                if eval > opt_eval {
                    opt_eval = eval;
                    next_tako = tako[i];
                    next_tako_i = i;
                }
            }

            // 次の目的地を決める
            let mut next_dest = dest[0];
            let mut opt_eval = f64::MIN;
            for i in 0..self.m {
                // コスト算出
                // たこ焼きから目的地までの移動
                let dist = self.dist(&next_tako, &dest[i]);
                let mut cost = (if dist > arm.long { dist - arm.long } else { 0 }).max(2);
                // 目的地からたこ焼きの重心までの移動
                let dist = self.dist(&dest[i], &s_cm);
                cost += (if dist > arm.long { dist - arm.long } else { 0 }).max(2);

                // 利益算出
                let (x, y) = dest[i];
                let benefit = if !self.s[x as usize][y as usize] { self.each_debt(&dest[i], &t_cm, &dir_st) } else { 0 };

                let eval = benefit as f64 / cost as f64;
                if eval > opt_eval {
                    opt_eval = eval;
                    next_dest = dest[i];
                }
            }

            let target = next_dest;
            let start = next_tako;
            // println!("start: {:?}, target: {:?}", start, target);
            let ops = arm.r#move(start, &target);
            self.op.extend(ops);
            if start == arm.leaf {
                self.s[start.0 as usize][start.1 as usize] = false;
            }
            // println!("get: {:?}", arm.leaf);
            let next = (0, 0);
            let ops = arm.r#move(target, &next);
            self.op.extend(ops);
            if target == arm.leaf {
                self.s[target.0 as usize][target.1 as usize] = true;
                tako[next_tako_i] = target;
            }
            // println!("put: {:?}", arm.leaf);
        }
        
    }

    fn debt(&self) -> usize {
        // 目的地の重心からのたこ焼きの距離の合計を負債とみなす
        // ただしtと一致している場合は負債とみなさない
        // sの重心から見て、s->tの方向と真逆は4倍、両脇は2倍、同じ方向は1倍の重みを付ける
        let mut debt: usize = 0;

        let (s_cm, _) = self.center(&self.s, &self.t);
        let s_cm = (s_cm.0 as i8, s_cm.1 as i8);
        for i in 0..self.n {
            for j in 0..self.n {
                let (x, y) = (i as i8, j as i8);
                if self.s[i][j] && !self.t[i][j] {
                    debt += self.each_debt(&(x, y), &s_cm, &s_cm);
                }
            }
        }

        debt
    }

    fn solve_v1(&mut self) {
        // たこ焼きの処理順を決定
        let (s_center, _) = self.center(&self.s, &self.t);
        let s_center= (s_center.0 as i8, s_center.1 as i8);
        // println!("center of s: {:?}", s_center);
        let (t_center, _) = self.center(&self.t, &self.s);
        let t_center= (t_center.0 as i8, t_center.1 as i8);
        // println!("center of t: {:?}", t_center);
        let start_dir: (i8, i8) = (
            if s_center.0 < t_center.0 { 1 } else { -1 },
            if s_center.1 < t_center.1 { 1 } else { -1 },
        );
        let sign = start_dir.0*start_dir.1;
        let mut s_tako: VecDeque<(i8, i8)> = VecDeque::new();
        let mut t_tako: VecDeque<(i8, i8)> = VecDeque::new();
        let mut order: Vec<(usize, usize)> = Vec::new();
        for sum in 0..(2*self.n-1) {
            for i in 0..=sum.min(self.n-1) {
                if sign < 0 && sum+1 > i+self.n { continue; }
                let j = if sign > 0 { sum - i } else { i+self.n-1-sum };
                if j < self.n {
                    order.push((i, j));
                }
            }
        }
        if start_dir.0 < 0 { order.reverse(); }
        for &(i, j) in order.iter() {
            // すでに目的地にたこ焼きがある場合は、処理対象から外す
            if self.s[i][j] && self.t[i][j] {
                continue;
            }
            if self.s[i][j] { s_tako.push_front((i as i8, j as i8)); }
            if self.t[i][j] { t_tako.push_front((i as i8, j as i8)); }
        }

        // アームの初期化
        (self.x, self.y) = s_center;
        let mut arm = RobotArm::new(self.n, (self.x, self.y), self.v);
        self.v2 = arm.v2;
        self.p_l.clone_from(&arm.p_l);

        // 処理
        while !s_tako.is_empty() {
            let start = s_tako.pop_back().unwrap();
            let target = t_tako.pop_back().unwrap();
            // println!("start: {:?}, target: {:?}", start, target);
            let ops = arm.r#move(start, &start);
            self.op.extend(ops);
            if start == arm.leaf {
                self.s[start.0 as usize][start.1 as usize] = false;
            }
            // println!("get: {:?}", arm.leaf);
            let ops = arm.r#move(target, &target);
            self.op.extend(ops);
            if target == arm.leaf {
                self.s[target.0 as usize][target.1 as usize] = true;
            }
            // println!("put: {:?}", arm.leaf);
        }
        
    }

    fn solve_v2(&mut self) {
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
            let ops = arm.r#move(start, &target);
            self.op.extend(ops);
            if start == arm.leaf {
                self.s[start.0 as usize][start.1 as usize] = false;
            }
            // println!("get: {:?}", arm.leaf);
            let next = if i < self.n-1 { self.decide_start(&t_tako[i+1], &fixed)} else { (0, 0) };
            let ops = arm.r#move(target, &next);
            self.op.extend(ops);
            if target == arm.leaf {
                self.s[target.0 as usize][target.1 as usize] = true;
                fixed[target.0 as usize][target.1 as usize] = true;
            }
            // println!("put: {:?}", arm.leaf);
        }
        
    }

    fn solve_v3(&mut self) {
        // たこ焼きと目的地の場所
        let mut tako: Vec<(i8, i8)> = self.pos(&self.s);
        let dest: Vec<(i8, i8)> = self.pos(&self.t);

        // アームの初期化
        let (s_cm, s_cnt) = self.center(&self.s, &self.t);
        let s_cm = (s_cm.0 as i8, s_cm.1 as i8);
        (self.x, self.y) = s_cm;
        let mut arm = RobotArm::new(self.n, (self.x, self.y), self.v);
        self.v2 = arm.v2;
        self.p_l.clone_from(&arm.p_l);

        // 処理
        for _ in 0..self.m {
            // 重心を算出
            let (s_cm, s_cnt) = self.center(&self.s, &self.t);
            let s_cm = (s_cm.0 as i8, s_cm.1 as i8);
            if s_cnt == 0 { break; }
            let (t_cm, t_cnt) = self.center(&self.t, &self.s);
            let t_cm = (t_cm.0 as i8, t_cm.1 as i8);
            let dir_st = self.dir(&s_cm, &t_cm);

            // 次のたこ焼きを決定
            let mut next_tako_i = 0;
            let mut next_tako = tako[next_tako_i];
            let mut opt_eval = f64::MIN;
            for i in 0..self.m {
                // コスト算出
                // たこ焼きまでの移動
                let dist = self.dist(&arm.root, &tako[i]);
                let mut cost = (if dist > arm.long { dist - arm.long } else { 0 }).max(2);
                // たこ焼きから目的地の重心までの移動
                let dist = self.dist(&tako[i], &t_cm);
                cost += (if dist > arm.long { dist - arm.long } else { 0 }).max(2);

                // 利益算出
                let (x, y) = tako[i];
                let benefit = if !self.t[x as usize][y as usize] { self.each_debt(&tako[i], &s_cm, &dir_st) } else { 0 };

                let eval = benefit as f64 / cost as f64;
                // println!("i: {}, tako: {:?}, eval: {}, benefit: {}, cost: {}", i, tako[i], eval, benefit, cost);
                if eval > opt_eval {
                    opt_eval = eval;
                    next_tako = tako[i];
                    next_tako_i = i;
                }
            }

            // 次の目的地を決める
            let mut next_dest = dest[0];
            let mut opt_eval = f64::MIN;
            for i in 0..self.m {
                // コスト算出
                // たこ焼きから目的地までの移動
                let dist = self.dist(&next_tako, &dest[i]);
                let mut cost = (if dist > arm.long { dist - arm.long } else { 0 }).max(2);
                // 目的地からたこ焼きの重心までの移動
                let dist = self.dist(&dest[i], &s_cm);
                cost += (if dist > arm.long { dist - arm.long } else { 0 }).max(2);

                // 利益算出
                let (x, y) = dest[i];
                let benefit = if !self.s[x as usize][y as usize] { self.each_debt(&dest[i], &t_cm, &dir_st) } else { 0 };

                let eval = benefit as f64 / cost as f64;
                if eval > opt_eval {
                    opt_eval = eval;
                    next_dest = dest[i];
                }
            }

            let target = next_dest;
            let start = next_tako;
            // println!("start: {:?}, target: {:?}", start, target);
            let ops = arm.r#move(start, &target);
            self.op.extend(ops);
            if start == arm.leaf {
                self.s[start.0 as usize][start.1 as usize] = false;
            }
            // println!("get: {:?}", arm.leaf);
            let next = (0, 0);
            let ops = arm.r#move(target, &next);
            self.op.extend(ops);
            if target == arm.leaf {
                self.s[target.0 as usize][target.1 as usize] = true;
                tako[next_tako_i] = target;
            }
            // println!("put: {:?}", arm.leaf);
        }
        
    }

    fn pos(&self, s: &Vec<Vec<bool>>) -> Vec<(i8, i8)> {
        let mut pos: Vec<(i8, i8)> = Vec::new();

        for i in 0..self.n {
            for j in 0..self.n {
                if s[i][j] {
                    pos.push((i as i8, j as i8));
                }
            }
        }

        pos
    }

    fn each_debt(&self, s: &(i8, i8), cm: &(i8, i8), dir_st: &(i8, i8)) -> usize {
        if s == cm { return 1; }
        let dir = self.dir(cm, s);
        let dot = (dir.0*dir_st.0) + (dir.1*dir_st.1);
        let w = if dot < 0 { 4 } else if dot == 0 { 2 } else { 1 };
        w * self.dist(s, cm)
        
    }

    fn dir(&self, s: &(i8, i8), t: &(i8, i8)) -> (i8, i8) {
        // sからtの方向を1, 0, -1で算出
        let x = if s.0 < t.0 { 1 } else if s.0 > t.0 { -1 } else { 0 };
        let y = if s.1 < t.1 { 1 } else if s.1 > t.1 { -1 } else { 0 };
        
        (x, y)
    }

    fn decide_start(&self, target: &(i8, i8), fixed: &[Vec<bool>]) -> (i8, i8) {
        let mut dist = self.n*2;
        let (mut x, mut y) = (0, 0);
        let mut dist2 = self.n*2;
        let (mut x2, mut y2) = (0, 0);
        for i in 0..self.n {
            for j in 0..self.n {
                if !self.s[i][j] { continue; }
                if fixed[i][j] { continue; }
                if self.t[i][j] {
                    let tmp_dist = self.dist(target, &(i as i8, j as i8));
                    if dist2 > tmp_dist {
                        dist2 = tmp_dist;
                        (x2, y2) = (i, j);
                    }
                    continue;
                }
                let tmp_dist = self.dist(target, &(i as i8, j as i8));
                if dist > tmp_dist {
                    dist = tmp_dist;
                    (x, y) = (i, j);
                }
            }
        }
        if (dist*2).max(4) > (dist2*2).max(4)+16 {
            (x, y) = (x2, y2);
        }

        (x as i8, y as i8)
    }

    fn dist(&self, p1: &(i8, i8), p2: &(i8, i8)) -> usize {
        (p1.0.abs_diff(p2.0) + p1.1.abs_diff(p2.1)) as usize
    }

    fn center(&self, s: &Vec<Vec<bool>>, t: &Vec<Vec<bool>>) -> ((f64, f64), usize) {
        let (mut c_x, mut c_y) = (0.0, 0.0);
        let mut cnt = 0;
        for x in 0..self.n {
            for y in 0..self.n {
                // たこ焼きと目的地が一致していない場合のみ重心を算出
                if s[x][y] && !t[x][y] {
                    c_x += x as f64;
                    c_y += y as f64;
                    cnt += 1;
                }
            }
        }
        if cnt == 0 { return ((-1.0, -1.0), 0); }
        c_x /= cnt as f64;
        c_y /= cnt as f64;

        ((c_x, c_y), cnt)
    }

    fn score(&self) -> usize {
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
        score
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

        let score = self.score();
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
    
    let mut run_solver = solver.clone();
    run_solver.solve();
    let mut opt_score = run_solver.score();
    let mut opt_solver = run_solver;

    let mut run_solver = solver.clone();
    run_solver.solve_v1();
    if opt_score > run_solver.score() {
        opt_score = run_solver.score();
        opt_solver = run_solver;
    }

    let mut run_solver = solver.clone();
    run_solver.solve_v2();
    if opt_score > run_solver.score() {
        opt_score = run_solver.score();
        opt_solver = run_solver;
    }
    
    let mut run_solver = solver.clone();
    run_solver.solve_v3();
    if opt_score > run_solver.score() {
        opt_score = run_solver.score();
        opt_solver = run_solver;
    }

    opt_solver.ans();
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
        let mut ops = arm.r#move((1, 2), &(0, 0));
        assert_eq!(arm.leaf, (1, 2));
        assert_eq!(ops.len(), 1);
        let mut ops = arm.r#move((3, 0), &(0, 0));
        assert_eq!(arm.leaf, (3, 0));
        assert_eq!(ops.len(), 1);
        let mut ops = arm.r#move((6, 0), &(0, 0));
        assert_eq!(arm.leaf, (6, 0));
        println!("end (6, 0)");
        let mut ops = arm.r#move((0, 0), &(0, 0));
        println!("end (0, 0)");
        assert_eq!(arm.leaf, (0, 0));
    }

}
