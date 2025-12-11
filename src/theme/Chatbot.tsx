import React, { useState, useEffect, useRef } from 'react';
import styles from './Chatbot.module.css';

const Chatbot = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [loading, setLoading] = useState(false);
  const apiUrl = (() => {
    const envUrl =
      (typeof process !== 'undefined' && process.env?.CHAT_API_URL) ||
      (typeof process !== 'undefined' && process.env?.REACT_APP_CHAT_API_URL);
    if (envUrl) return envUrl.replace(/\/$/, '') + '/api/chat';
    if (typeof window !== 'undefined' && window.location.origin.includes('localhost:3000')) {
      // In local dev, point to FastAPI backend
      return 'http://localhost:8000/api/chat';
    }
    // Default to Hugging Face Space backend
    return 'https://mirza-rehan76-rag-chatbot.hf.space/api/chat';
  })();
  const messagesEndRef = useRef(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  useEffect(scrollToBottom, [messages]);

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const handleInputChange = (e) => {
    setInputValue(e.target.value);
  };

  const tryFetch = async (url: string) => {
    const response = await fetch(url, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({ query: inputValue }),
    });

    if (!response.ok) {
      const text = await response.text();
      throw new Error(text || `Request failed: ${response.status}`);
    }

    return response.json();
  };

  const handleSendMessage = async () => {
    if (inputValue.trim() && !loading) {
      const userMessage = { text: inputValue, sender: 'user' };
      setMessages((prevMessages) => [...prevMessages, userMessage]);
      setInputValue('');
      setLoading(true);

      try {
        let data = await tryFetch(apiUrl);

        // Fallback: if first attempt fails due to network/CORS, try local FastAPI
        // Only attempt when not already pointing at localhost:8000
        if (!data && typeof window !== 'undefined' && !apiUrl.includes('localhost:8000')) {
          data = await tryFetch('http://localhost:8000/api/chat');
        }

        const botMessage = { text: data.response, sender: 'bot' };
        setMessages((prevMessages) => [...prevMessages, botMessage]);
      } catch (error) {
        console.error('Error fetching data:', error);
        const errorText = (error as Error)?.message || 'Failed to reach chatbot service. Ensure the backend is running.';
        const errorMessage = { text: errorText, sender: 'bot', isError: true };
        setMessages((prevMessages) => [...prevMessages, errorMessage]);
      } finally {
        setLoading(false);
      }
    }
  };

  return (
    <>
      <button className={styles.chatToggleButton} onClick={toggleChat} title="Open chatbot">
        {isOpen ? '✕' : '💬'}
      </button>
      {isOpen && (
        <div className={styles.chatWindow}>
          <div className={styles.chatHeader}>
            <h2>AI Assistant</h2>
            <button className={styles.closeButton} onClick={toggleChat} title="Close">✕</button>
          </div>
          <div className={styles.chatMessages}>
            {messages.map((msg, index) => (
              <div key={index} className={`${styles.message} ${styles[msg.sender]} ${msg.isError ? styles.error : ''}`}>
                {msg.text}
              </div>
            ))}
            {loading && <div className={`${styles.message} ${styles.bot}`}>Thinking...</div>}
            <div ref={messagesEndRef} />
          </div>
          <div className={styles.chatInput}>
            <input
              type="text"
              value={inputValue}
              onChange={handleInputChange}
              onKeyPress={(e) => e.key === 'Enter' && handleSendMessage()}
              placeholder={loading ? "Please wait..." : "Ask a question..."}
              disabled={loading}
            />
            <button onClick={handleSendMessage} disabled={loading}>Send</button>
          </div>
        </div>
      )}
    </>
  );
};

export default Chatbot;
