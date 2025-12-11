import React, { useState, useRef, useEffect } from 'react';
import { Chapter } from '../../types/Chapter';

interface AITutorProps {
  context?: string; // Context for the AI tutor (e.g., current chapter title)
  chapter?: Chapter; // Optional chapter for context-aware assistance
}

interface Message {
  id: string;
  text: string;
  sender: 'user' | 'tutor';
  timestamp: Date;
  rating?: number; // User rating of the response (1-5)
}

const AITutor: React.FC<AITutorProps> = ({ context, chapter }) => {
  const [messages, setMessages] = useState<Message[]>([]);
  const [inputText, setInputText] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const messagesEndRef = useRef<null | HTMLDivElement>(null);

  // Load initial greeting message
  useEffect(() => {
    setMessages([
      {
        id: 'initial',
        text: `Hello! I'm your AI tutor. ${
          context 
            ? `I can help you with ${context}. Ask me anything!` 
            : 'Ask me anything about the topic!'
        }`,
        sender: 'tutor',
        timestamp: new Date(),
      }
    ]);
  }, [context]);

  // Scroll to bottom of messages
  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    
    if (!inputText.trim() || isLoading) return;

    // Add user message
    const userMessage: Message = {
      id: Date.now().toString(),
      text: inputText,
      sender: 'user',
      timestamp: new Date(),
    };
    
    setMessages(prev => [...prev, userMessage]);
    setInputText('');
    setIsLoading(true);
    setError(null);

    try {
      // In a real implementation, this would call the AI tutor API
      // Simulating API call for demonstration
      await new Promise(resolve => setTimeout(resolve, 1000));
      
      // Mock AI response (in a real implementation, this would come from the API)
      const aiResponse: Message = {
        id: (Date.now() + 1).toString(),
        text: `I understand you're asking about "${inputText}". As an AI tutor for Physical AI & Humanoid Robotics, I can help explain concepts related to this field. ${
          chapter 
            ? `Based on the current chapter "${chapter.title}", I can provide more specific information.` 
            : 'Please provide more context for a better answer.'
        }`,
        sender: 'tutor',
        timestamp: new Date(),
      };
      
      setMessages(prev => [...prev, aiResponse]);
    } catch (err) {
      setError('Failed to get response from AI tutor. Please try again.');
      console.error('Error getting AI response:', err);
    } finally {
      setIsLoading(false);
    }
  };

  const rateResponse = async (messageId: string, rating: number) => {
    // In a real implementation, this would call the API to rate the response
    console.log(`Rating message ${messageId} with ${rating} stars`);
    
    setMessages(prev => 
      prev.map(msg => 
        msg.id === messageId 
          ? { ...msg, rating } 
          : msg
      )
    );
  };

  return (
    <div className="flex flex-col h-full border rounded-lg overflow-hidden bg-white">
      <div className="bg-blue-600 text-white p-3">
        <h3 className="font-semibold">AI Tutor</h3>
        <p className="text-xs opacity-80">
          {context ? `Context: ${context}` : 'Ask anything about Physical AI & Robotics'}
        </p>
      </div>
      
      <div className="flex-grow overflow-y-auto p-3 space-y-3 max-h-96">
        {messages.map((message) => (
          <div 
            key={message.id} 
            className={`flex ${message.sender === 'user' ? 'justify-end' : 'justify-start'}`}
          >
            <div 
              className={`max-w-[80%] rounded-lg px-3 py-2 text-sm ${
                message.sender === 'user' 
                  ? 'bg-blue-500 text-white rounded-tr-none' 
                  : 'bg-gray-200 text-gray-800 rounded-tl-none'
              }`}
            >
              <div className="text-xs opacity-80 mb-1">
                {message.sender === 'user' ? 'You' : 'AI Tutor'} •{' '}
                {message.timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
              </div>
              <div>{message.text}</div>
              
              {message.sender === 'tutor' && message.rating === undefined && (
                <div className="mt-2 flex space-x-1">
                  {[1, 2, 3, 4, 5].map((star) => (
                    <button
                      key={star}
                      onClick={() => rateResponse(message.id, star)}
                      className="text-yellow-400 hover:text-yellow-500"
                      title={`Rate ${star} star${star !== 1 ? 's' : ''}`}
                    >
                      ★
                    </button>
                  ))}
                  <span className="text-xs text-gray-500 ml-1">Rate</span>
                </div>
              )}
              
              {message.rating !== undefined && (
                <div className="mt-1 text-xs text-gray-600">
                  {'★'.repeat(message.rating)}{'☆'.repeat(5 - message.rating)} (rated)
                </div>
              )}
            </div>
          </div>
        ))}
        
        {isLoading && (
          <div className="flex justify-start">
            <div className="bg-gray-200 text-gray-800 rounded-lg px-3 py-2 text-sm rounded-tl-none max-w-[80%]">
              <div className="flex items-center">
                <div className="animate-pulse">AI Tutor is typing...</div>
                <div className="ml-2">
                  <div className="h-2 w-2 bg-gray-500 rounded-full animate-bounce" style={{ animationDelay: '0ms' }}></div>
                  <div className="h-2 w-2 bg-gray-500 rounded-full animate-bounce" style={{ animationDelay: '150ms' }}></div>
                  <div className="h-2 w-2 bg-gray-500 rounded-full animate-bounce" style={{ animationDelay: '300ms' }}></div>
                </div>
              </div>
            </div>
          </div>
        )}
        
        <div ref={messagesEndRef} />
      </div>
      
      <form onSubmit={handleSubmit} className="border-t p-3">
        {error && (
          <div className="text-red-500 text-xs mb-2">{error}</div>
        )}
        
        <div className="flex">
          <input
            type="text"
            value={inputText}
            onChange={(e) => setInputText(e.target.value)}
            placeholder="Ask your AI tutor..."
            className="flex-grow border border-gray-300 rounded-l-md px-3 py-2 text-sm focus:outline-none focus:ring-1 focus:ring-blue-500"
            disabled={isLoading}
          />
          <button
            type="submit"
            disabled={!inputText.trim() || isLoading}
            className="bg-blue-600 text-white px-4 py-2 rounded-r-md text-sm disabled:opacity-50"
          >
            Send
          </button>
        </div>
        
        <p className="text-xs text-gray-500 mt-1">
          AI Tutor can help explain concepts, clarify doubts, and provide additional information related to Physical AI & Humanoid Robotics.
        </p>
      </form>
    </div>
  );
};

export default AITutor;