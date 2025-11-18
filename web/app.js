require('dotenv').config();

const express = require('express');
const mysql = require('mysql');

const app = express();
const port = 3000;

// EJS 템플릿 엔진 설정
app.set('view engine', 'ejs');
app.use(express.urlencoded({ extended: false }));

const connection = mysql.createConnection({
  host: process.env.DB_HOST,
  user: process.env.DB_USER,
  password: process.env.DB_PASSWORD,
  database: process.env.DATABASE
});

connection.connect((err) => {
  if (err) {
    console.error('MySQL connection Fail:', err);
    process.exit(1);
  }
  console.log('MySQL connection Success');
});

app.get('/', (req, res) => {
  connection.query('SELECT * FROM tb_robot', (err, robotResults) => {
    if (err) {
      console.error(err);
      return res.status(500).send('서버 오류');
    }

    connection.query('SELECT taskName FROM tb_task', (err, taskResults) => {
      if (err) {
        console.error(err);
        return res.status(500).send('서버 오류');
      }

      res.render('index', { robots: robotResults, tasks: taskResults });
    });
  });
});

// 모든 로봇 상태 idle
app.post('/idle', (req, res) => {
  const sql = `
    UPDATE tb_robot
    SET situation = 'Idle',
        task = NULL,
        state = 'idle'
  `;
  connection.query(sql, (err) => {
    if (err) {
      console.error(err);
      return res.status(500).send('서버 오류');
    }
    res.redirect('/');
  });
});

// 모든 로봇 상태 powerOff
app.post('/powerOff', (req, res) => {
  const sql = `
    UPDATE tb_robot
    SET situation = 'PowerOff',
        task = NULL,
        state = 'powerOff'
  `;
  connection.query(sql, (err) => {
    if (err) {
      console.error(err);
      return res.status(500).send('서버 오류');
    }
    res.redirect('/');
  });
});

// robotID의 situation 수정
app.post('/updateSituationById', (req, res) => {
  const robotId = parseInt(req.body.robotId);
  const situation = req.body.situation;

  if (!robotId || !situation) {
    return res.status(400).send('유효하지 않은 입력입니다.');
  }

  const query = 'UPDATE tb_robot SET situation = ? WHERE idRobot = ?';
  connection.query(query, [situation, robotId], (err, result) => {
    if (err) {
      console.error(err);
      return res.status(500).send('DB 오류');
    }
    if (result.affectedRows === 0) {
      return res.status(404).send('해당 ID의 로봇이 없습니다.');
    }
    res.redirect('/');
  });
});

// robotID의 task 수정
app.post('/updateTaskById', (req, res) => {
  const { robotId, task } = req.body;
  const query = 'UPDATE tb_robot SET task = ? WHERE idRobot = ?';
  connection.query(query, [task || null, robotId], (err) => {
    if (err) return res.status(500).send('DB 오류');
    res.redirect('/');
  });
});

// robotID의 state 수정
app.post('/updateStateById', (req, res) => {
  const { robotId, state } = req.body;
  const query = 'UPDATE tb_robot SET state = ? WHERE idRobot = ?';
  connection.query(query, [state, robotId], (err) => {
    if (err) return res.status(500).send('DB 오류');
    res.redirect('/');
  });
});

app.listen(port, () => {
  console.log(`Server is running on http://localhost:${port}`);
});

// app.listen(port, '0.0.0.0', () => {
//   console.log(`Server is running on http://0.0.0.0:${port}`);
// });