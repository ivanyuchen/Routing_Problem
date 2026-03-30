# Lab1 - Channel Routing

**Deadline:** 4/5/2026 23:59 

## Problem Description

Implement a 2-layer detailed router to complete channel routing problems. You must use the **greedy channel routing algorithm**.


## Routing Rules and Costs

* **Vias:** A via is created at the intersection of one horizontal and one vertical wire segment of the same net.


* **Via Cost:** 5.


* **Top/Bottom constraints:** Horizontal wire segments cannot be placed on the top or bottom rows (e.g., `.H 0 0 1` is illegal).

* **Short:** A "short error" occurs if two wire segments of different nets overlap in the same direction.


* **Coupling Length:** In channel routing, consider two nets $n_i$ and $n_j$. If they occupy adjacent tracks, the **coupling length** is defined as the length of this parallel segment.
  - If two adjacent tracks of a net both contain wire segments, both sides must be accounted for.
  - Only the horizontal portions are calculated as **coupling length**.

* **Spill-over:** Spilling are only allowed on the **rightside**.



## Provided Tools

* **Plotter:** A Python script is provided for visual debugging:


  `python3 plotter.py --in_file [input.txt] --out_file [output.txt] --img_name [name.png]`.


* **Makefile:** A template is provided. Your final `make` command must generate an executable named **Lab1**. Feel free to modify the Makefile as needed.

* **output2csv:** Convert the routing results into a CSV format suitable for Kaggle submission.
 `./output2csv [output_name.txt] [csv_name.csv]`


## Grading Policy

Your score is primarily evaluated based on the testcase **"testcase1.txt / testcase2.txt"**.
- **Runtime Limit:** 300 seconds (Failure to finish results in 0 points).
- **Compiler Version**: gcc 8.5.0
- **Multithread** is not allowed.
#### penalties:
- **Spill-over Area:** Spilling are only allowed, but penalized by the width of the spill-over region: $\text{penalty} = \text{width of spill-over region}$.
- **Coupling Length Penalty**: the **coupling length ratio** for each net is considered as the **coupling length** devides by **net span**(the length from leftmost pin to rightmost pin)
$\text{penalty} = max_{\forall  \text{coupling pair}}(coupling\_length\_ratio)$


1. Testcase 1(50%)

   - **<span style="color: red;">Baseline: 30 tracks</span>**

   | Coupling Score Range  | Tracks Range | Score Region |
   |-----------------------|--------------|--------------|
   | [0, 0.2)              | <=30         | [85, 100]    |
   | [0.2,0.5]             | <=30         | [75, 85)     |
   | (0.5, inf)            | <=30         | [65, 75)     |
   | -                     | >30          | 0            |


2. Testcase 2(50%)

   - **<span style="color: red;">Baseline: 60 tracks</span>**

   | Coupling Score Range  | Tracks Range | Score Region |
   |-----------------------|--------------|--------------|
   | [0, 0.2)              | <=60         | [85, 100]    |
   | [0.2,0.5]             | <=60         | [75, 85)     |
   | (0.5, inf)            | <=60         | [65, 75)     |
   | -                     | >60          | 0            |



- Achieving the baseline get **65%** score.

- The final **35%** will be based on ranking. Note: **Only solutions surpassing the baseline and submitted before last submission date(4/19) will be considered for competition**.

- Your score for this lab is the average of testcase 1 score and testcase 2 score. 

* **Ranking Policy**: The ranking is based on the following formula
    - $Cost = \alpha \times exp(trackNum/\delta) + \beta \times (WL + 5 \times \#\ of\ via) + \gamma \times SpillOverTrack$
    - $\alpha = 10$ 
    - $\beta = 0.00001$ 
    - $\gamma = 5$
    - $\delta = 20$ for case 1, $40$ for case2

- For a score range containing $n$ students, if your rank within that range is $k$, your final grade will be:

$FinalScore = ScoreRegionMin + (ScoreRegionMax - ScoreRegionMin) \times k/ n$

<span style="color: red;">Any adjustments to the scoring method will be announced immediately.</span>

## Input/Output Format

### Input:
The first two lines contrain of the input file consists rows of integers. Each number represents the **net id** of the pin (except for 0). All pins in the top and bottom rows with the same net id must be connected together.
Next line contains a single integer $n$, means there are $n$ following coupling constraint. For each constraint, there are two integers represent net $n_i$, $n_j$, means you need to consider coupling constraint from $n_j$ to $n_i$. 

```text
<alpha> <beta> <gamma> <delta>
<top pin order>
<bottom pin order>
<constraint num>
<net a1> <net b1> # contraint 1
...
<net an> <net bn> # contraint n
```

**Example Input:**

```text
10 0.00001 5 20
0 1 3 2 11 5 3 1 0
1 5 11 5 1 1 4 2 4
2
1 5
5 1
```
![螢幕擷取畫面 2026-02-28 021447](https://hackmd.io/_uploads/HJIFN2xKbx.png)



### Output:
Print all horizontal and vertical segments of every net to a file.

* **Column width and track height:** 1.
* **Bottom layer coordinate:** Starts from 0.



**Format:**

```text
.begin net_name
.H lef_x lef_y rig_x
.V bot_x bot_y top_y
.end
```

**Example Output (for Net 4):**

```text
.begin 4 
.H 0 1 2 
.V 0 0 1 
.V 1 0 1 
.V 2 1 2 
.H 2 2 3 
.V 3 2 3 
.end 
```
![螢幕擷取畫面 2026-02-28 021645](https://hackmd.io/_uploads/SkchE2eK-g.png)







## Submission Requirements

Submit a `.tar` file containing your source code and Makefile to E3.

```bash=
$ tar cvf studentID.tar studentID
```
        
We expect your file can be executed after
```bash=
$ tar xvf studentID.tar
$ cd studentID
$ ls // should show Makefile, and your src
$ make -j
$./Lab1 [input.txt] [output.txt]
```

#### Late Penalties: 
- 4/6 to 4/12: $Score \times 0.95$.
- 4/13 to 4/19: $Score \times 0.8$.
- After 4/20: 0 points.


#### Wrong Format: -10 points.


#### Fraud: 0 points.

- You may use any AI tools to assist in completing this. 
- <span style="color: red;">Plagiarism of any kind is **strictly prohibited**, including but not limited to copying existing code from the internet or from other students. If plagiarism is detected, both the provider and the recipient will receive 0 points for this assignment.</span>


## Scoreboard
- To make the rankings more exciting, we’ve set up Kaggle competitions [Testcase1](https://www.kaggle.com/t/be94fe676f514d7cb67f15c4449b0c5c) and [Testcase2](https://www.kaggle.com/t/4cb7bf598277404490a19a26c36c07ec) ! You can upload your results and see everyone else’s performance in real-time.

- Please note: The Kaggle leaderboard is for reference only. Your final grade will be based on the TA's official execution of your final submission.

- Please use your **Student ID** as team name to join the Kaggle competition.

- To separate different score level and display on same scoreboard, different level will add additional base score (Use testcase 1 for example):

  | Coupling Length Range | Tracks Range | Kaggle Score |
  |-----------------------|--------------|--------------|
  | [0, 0.2)              | <=30         | score        |
  | [0.2,0.5]             | <=30         | score+10000  |
  | (0.5, inf)            | <=30         | score+20000  |
  | -                     | >30          | $10^9$       |
  
## Contact Information
If you have any questions regarding the lab, please feel free to contact the TAs.

- Please use the E3 platform email system.
- You MUST **CC all TAs** in your email to ensure a timely response.

## Q&A
Before asking a question, please check this section to see if it has already been answered. This section will be updated dynamically.

- **Q1:** I would like to ask if this assignment can be written in Python or C++. If Python is allowed, is it okay to use a virtual environment manager like 'uv'?
**A1:** The assignment is restricted to C/C++. Our default Makefile is configured for C++17; however, you are free to modify it as needed. Please ensure that the final output format after running `make` strictly follows the specifications in the documentation.

- **Q2:** According to the specs, is the grade determined solely by testcase1 and testcase2? Are there any other hidden test cases for grading?
**A2:** There are no other hidden test cases, so the grade is based only on testcase1 and testcase2.

- **Q3:** Is it mandatory to merge output segments into their simplest form? For example, can I output `.H 0 1 2` as two separate segments: `.H 0 1 1` and `.H 1 1 2`?
**A3:** That is acceptable. You don't need to merge them into the simplest form; however, be aware that any redundant segments will lead to extra WL calculations.