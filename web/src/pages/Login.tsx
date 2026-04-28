import { useState, type FormEvent } from "react";
import { Link, useNavigate, useSearchParams } from "react-router-dom";
import { login } from "../api/auth";
import logo from "../assets/logo.png";
import "./Auth.css";

export default function Login() {
  const navigate = useNavigate();
  const [searchParams] = useSearchParams();
  const [email, setEmail] = useState("");
  const [password, setPassword] = useState("");
  const [error, setError] = useState("");

  const fromRobot = searchParams.get("from") === "robot";

  const handleSubmit = async (e: FormEvent) => {
    e.preventDefault();
    try {
      const { access_token, refresh_token } = await login(email, password);
      localStorage.setItem("access_token", access_token);
      localStorage.setItem("refresh_token", refresh_token);
      if (fromRobot) {
        const res = await fetch("/api/v1/auth/issue-code", {
          method: "POST",
          headers: { Authorization: `Bearer ${access_token}` },
        });
        const { code } = await res.json();
        navigate(`/auth/callback?code=${code}`);
      } else {
        navigate("/");
      }
    } catch (err) {
      setError(err instanceof Error ? err.message : "로그인에 실패했습니다");
    }
  };

  return (
    <div className="auth-container">
      <div className="auth-card">
        <div className="auth-logo">
          <img src={logo} alt="Robot Studio" />
        </div>
        <h1>로그인</h1>
        <p className="subtitle">Robot Studio에 오신 것을 환영합니다</p>
        {error && <p className="error-message">{error}</p>}
        <form className="auth-form" onSubmit={handleSubmit}>
          <div className="form-group">
            <label htmlFor="email">이메일</label>
            <input
              id="email"
              type="email"
              placeholder="name@example.com"
              value={email}
              onChange={(e) => setEmail(e.target.value)}
              required
            />
          </div>
          <div className="form-group">
            <label htmlFor="password">비밀번호</label>
            <input
              id="password"
              type="password"
              placeholder="비밀번호를 입력하세요"
              value={password}
              onChange={(e) => setPassword(e.target.value)}
              required
            />
          </div>
          <button type="submit" className="auth-submit">
            로그인
          </button>
        </form>
        <div className="auth-footer">
          계정이 없으신가요?{" "}
          <Link to={fromRobot ? "/signup?from=robot" : "/signup"}>
            회원가입
          </Link>
        </div>
      </div>
    </div>
  );
}
