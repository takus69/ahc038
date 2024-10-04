use proconio::{input, marker::Chars};

struct Solver {
    n: usize,
    m: usize,
    v: usize,
    s: Vec<Vec<bool>>,
    t: Vec<Vec<bool>>,
    v2: usize,
    p_l: Vec<(usize, usize)>,
    x :usize,
    y: usize,
    op: Vec<String>,
}

impl Solver {
    fn solve(&self) {
        
    }

    fn ans(&self) {
        println!("{}", self.v2);
        for (p, l) in self.p_l.iter() {
            println!("{} {}", p, l);
        }
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
